//! Which NAV-PVT goes out as a fix, and when.
//!
//! An epoch is the NAV-PVT and, when enabled, the NAV-HPPOSLLH of one iTOW;
//! the receiver ends it with NAV-EOE. A receiver whose output buffer is full
//! drops messages ("txbuf alloc"), NAV-EOE first as the last of the epoch:
//! on XR-02 (2026-10-24) a flooded UART1 left no NAV-EOE at all on USB while
//! NAV-PVT and NAV-HPPOSLLH kept coming, and closing epochs on NAV-EOE only
//! published one fix in minutes. So an epoch closes
//!
//! - as soon as it has what the fix uses: the NAV-PVT, and the NAV-HPPOSLLH
//!   of the same iTOW when that is enabled;
//! - else on the NAV-EOE of its iTOW (its NAV-HPPOSLLH was lost);
//! - else on the next epoch's NAV-PVT or NAV-EOE, one epoch late (counted in
//!   [`Assembler::late`]), still stamped when its NAV-PVT arrived.
//!
//! With neither NAV-HPPOSLLH nor NAV-EOE (both refused by the receiver)
//! every NAV-PVT is an epoch of its own. No ROS here, so it is host-testable.

use std::time::SystemTime;

use mower_ubx::{NavHpPosLlh, NavPvt};

/// A closed epoch, ready to publish.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Epoch {
    pub pvt: NavPvt,
    /// The NAV-HPPOSLLH of the same iTOW, if it arrived.
    pub hp: Option<NavHpPosLlh>,
    /// When the NAV-PVT arrived: the fix's stamp.
    pub received_at: SystemTime,
}

pub struct Assembler {
    hp_enabled: bool,
    eoe_enabled: bool,
    /// The NAV-PVT of the epoch still open, with its arrival.
    pvt: Option<(NavPvt, SystemTime)>,
    /// A NAV-HPPOSLLH whose NAV-PVT has not arrived (yet).
    hp: Option<NavHpPosLlh>,
    /// Epochs closed late since the start.
    pub late: u64,
}

impl Assembler {
    /// `hp_enabled` / `eoe_enabled`: what the receiver acknowledged.
    pub fn new(hp_enabled: bool, eoe_enabled: bool) -> Self {
        Self { hp_enabled, eoe_enabled, pvt: None, hp: None, late: 0 }
    }

    /// NAV-PVT: the epoch still open before it, if nothing closed it, then
    /// this one if it needs nothing more.
    pub fn on_pvt(&mut self, pvt: NavPvt, received_at: SystemTime) -> impl Iterator<Item = Epoch> {
        let late = self.close_late(pvt.itow_ms);
        if self.hp.map(|h| h.itow_ms) != Some(pvt.itow_ms) {
            self.hp = None;
        }
        let complete = self.hp.is_some() || (!self.hp_enabled && !self.eoe_enabled);
        let now = if complete {
            Some(Epoch { pvt, hp: self.hp.take(), received_at })
        } else {
            self.pvt = Some((pvt, received_at));
            None
        };
        [late, now].into_iter().flatten()
    }

    /// NAV-HPPOSLLH: closes the open epoch of its iTOW.
    pub fn on_hp(&mut self, hp: NavHpPosLlh) -> Option<Epoch> {
        if !self.hp_enabled {
            return None;
        }
        match self.pvt {
            Some((pvt, received_at)) if pvt.itow_ms == hp.itow_ms => {
                self.pvt = None;
                Some(Epoch { pvt, hp: Some(hp), received_at })
            }
            _ => {
                self.hp = Some(hp);
                None
            }
        }
    }

    /// NAV-EOE: closes the open epoch, on time if it is the one of `itow`
    /// (its NAV-HPPOSLLH was lost), late if it is an older one (its own
    /// NAV-EOE and this epoch's NAV-PVT were lost).
    pub fn on_eoe(&mut self, itow: u32) -> Option<Epoch> {
        if !self.eoe_enabled {
            return None;
        }
        let closed = match self.pvt {
            Some((pvt, received_at)) if pvt.itow_ms == itow => {
                self.pvt = None;
                Some(Epoch { pvt, hp: self.take_hp(itow), received_at })
            }
            _ => self.close_late(itow),
        };
        // nothing of this epoch is still to come
        self.hp = None;
        closed
    }

    /// Close the open epoch late unless it is the one of `itow`; a second
    /// NAV-PVT of the same iTOW replaces the first.
    fn close_late(&mut self, itow: u32) -> Option<Epoch> {
        let (pvt, received_at) = self.pvt.take()?;
        if pvt.itow_ms == itow {
            return None;
        }
        self.late += 1;
        Some(Epoch { pvt, hp: self.take_hp(pvt.itow_ms), received_at })
    }

    fn take_hp(&mut self, itow: u32) -> Option<NavHpPosLlh> {
        if self.hp.map(|h| h.itow_ms) == Some(itow) {
            self.hp.take()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn pvt(itow: u32) -> NavPvt {
        let mut p = vec![0u8; NavPvt::LEN];
        p[0..4].copy_from_slice(&itow.to_le_bytes());
        NavPvt::parse(&p).unwrap()
    }

    fn hp(itow: u32) -> NavHpPosLlh {
        let mut p = vec![0u8; NavHpPosLlh::LEN];
        p[4..8].copy_from_slice(&itow.to_le_bytes());
        NavHpPosLlh::parse(&p).unwrap()
    }

    fn at(ms: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_millis(1_792_848_000_000 + ms)
    }

    /// (iTOW, hp present, stamp) of each epoch closed by the calls.
    fn closed(epochs: impl IntoIterator<Item = Epoch>) -> Vec<(u32, bool, SystemTime)> {
        epochs.into_iter().map(|e| (e.pvt.itow_ms, e.hp.is_some(), e.received_at)).collect()
    }

    #[test]
    fn full_epoch_closes_on_its_hpposllh_and_eoe_adds_nothing() {
        let mut a = Assembler::new(true, true);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(closed(a.on_hp(hp(1000))), vec![(1000, true, at(0))]);
        assert_eq!(a.on_eoe(1000), None);
        assert_eq!(a.late, 0);
    }

    #[test]
    fn epochs_keep_coming_without_any_eoe() {
        // XR-02, 2026-10-24: NAV-PVT and NAV-HPPOSLLH arrive, NAV-EOE never
        let mut a = Assembler::new(true, true);
        let mut out = Vec::new();
        for k in 0..8u32 {
            let itow = 1000 + 250 * k;
            out.extend(closed(a.on_pvt(pvt(itow), at(250 * k as u64))));
            out.extend(closed(a.on_hp(hp(itow))));
        }
        assert_eq!(out.len(), 8);
        assert!(out.iter().all(|&(_, with_hp, _)| with_hp));
        assert_eq!(a.late, 0);
    }

    #[test]
    fn lost_hpposllh_closes_on_eoe() {
        let mut a = Assembler::new(true, true);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(closed(a.on_eoe(1000)), vec![(1000, false, at(0))]);
        assert_eq!(a.late, 0);
    }

    #[test]
    fn epoch_without_hpposllh_and_eoe_closes_late_with_its_own_stamp() {
        let mut a = Assembler::new(true, true);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        // both lost; the next epoch's NAV-PVT closes it
        assert_eq!(closed(a.on_pvt(pvt(1250), at(250))), vec![(1000, false, at(0))]);
        assert_eq!(closed(a.on_hp(hp(1250))), vec![(1250, true, at(250))]);
        assert_eq!(a.late, 1);
    }

    #[test]
    fn eoe_of_a_later_epoch_closes_the_open_one_late() {
        let mut a = Assembler::new(true, true);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        // NAV-EOE 1000, NAV-PVT 1250 and NAV-HPPOSLLH 1250 lost: an orphan
        // NAV-HPPOSLLH, then NAV-EOE 1250
        assert_eq!(a.on_hp(hp(1250)), None);
        assert_eq!(closed(a.on_eoe(1250)), vec![(1000, false, at(0))]);
        assert_eq!(a.late, 1);
        // the orphan does not stick to the next epoch
        assert_eq!(closed(a.on_pvt(pvt(1500), at(500))), vec![]);
        assert_eq!(closed(a.on_eoe(1500)), vec![(1500, false, at(500))]);
    }

    #[test]
    fn hpposllh_before_its_pvt_closes_on_the_pvt() {
        let mut a = Assembler::new(true, true);
        assert_eq!(a.on_hp(hp(1000)), None);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![(1000, true, at(0))]);
        assert_eq!(a.on_eoe(1000), None);
    }

    #[test]
    fn stale_hpposllh_is_not_paired_with_a_newer_pvt() {
        let mut a = Assembler::new(true, true);
        assert_eq!(a.on_hp(hp(750)), None);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(closed(a.on_eoe(1000)), vec![(1000, false, at(0))]);
    }

    #[test]
    fn what_the_receiver_refused_is_not_waited_for() {
        // NAV-EOE only: on the NAV-EOE, and late without it
        let mut a = Assembler::new(false, true);
        assert_eq!(a.on_hp(hp(1000)), None);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(closed(a.on_eoe(1000)), vec![(1000, false, at(0))]);
        assert_eq!(closed(a.on_pvt(pvt(1250), at(250))), vec![]);
        assert_eq!(closed(a.on_pvt(pvt(1500), at(500))), vec![(1250, false, at(250))]);
        // NAV-HPPOSLLH only: on it, NAV-EOE ignored
        let mut a = Assembler::new(true, false);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(a.on_eoe(1000), None);
        assert_eq!(closed(a.on_hp(hp(1000))), vec![(1000, true, at(0))]);
        // neither: every NAV-PVT at once
        let mut a = Assembler::new(false, false);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![(1000, false, at(0))]);
        assert_eq!(closed(a.on_pvt(pvt(1250), at(250))), vec![(1250, false, at(250))]);
        assert_eq!(a.late, 0);
    }

    #[test]
    fn repeated_pvt_of_one_itow_keeps_the_newer() {
        let mut a = Assembler::new(true, true);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(0))), vec![]);
        assert_eq!(closed(a.on_pvt(pvt(1000), at(10))), vec![]);
        assert_eq!(closed(a.on_hp(hp(1000))), vec![(1000, true, at(10))]);
        assert_eq!(a.late, 0);
    }
}
