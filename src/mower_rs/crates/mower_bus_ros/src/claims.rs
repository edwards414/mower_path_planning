//! One direction, and one bridge, per name in the process (README.md, "One
//! direction per name").
//!
//! The registry is process-wide, not per bridge, because the echo it
//! prevents travels through DDS, and all of `mower_rsd` is one DDS
//! participant: a second bridge reading a name the first one writes would
//! take the first one's samples back onto the bus. Two bridges writing one
//! name would put every bus message on DDS twice.

use std::collections::BTreeMap;
use std::sync::{Mutex, MutexGuard, PoisonError};

use crate::{BridgeError, Direction, Kind};

/// Every name some live bridge holds, with its direction.
static CLAIMED: Mutex<BTreeMap<(Kind, String), Direction>> = Mutex::new(BTreeMap::new());

fn claimed() -> MutexGuard<'static, BTreeMap<(Kind, String), Direction>> {
    // Nothing panics under this lock; a poisoned map is still consistent.
    CLAIMED.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The names one bridge holds. Dropping it releases them, which the bridge
/// does once its DDS endpoints are gone.
#[derive(Debug, Default)]
pub(crate) struct Claims {
    held: Vec<(Kind, String)>,
}

impl Claims {
    /// Hold `name` for `direction`, or say who holds it.
    pub(crate) fn claim(&mut self, name: &str, direction: Direction) -> Result<(), BridgeError> {
        let key = (direction.kind(), name.to_string());
        let mut claimed = claimed();
        if let Some(&existing) = claimed.get(&key) {
            return Err(BridgeError::AlreadyBridged { name: name.to_string(), existing, requested: direction });
        }
        claimed.insert(key.clone(), direction);
        self.held.push(key);
        Ok(())
    }

    /// Give back a name whose endpoints could not be created.
    pub(crate) fn release(&mut self, name: &str, direction: Direction) {
        let key = (direction.kind(), name.to_string());
        if let Some(i) = self.held.iter().position(|held| *held == key) {
            self.held.swap_remove(i);
            claimed().remove(&key);
        }
    }
}

impl Drop for Claims {
    fn drop(&mut self) {
        let mut claimed = claimed();
        for key in self.held.drain(..) {
            claimed.remove(&key);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_name_is_held_once_per_kind_until_its_holder_goes() {
        let mut a = Claims::default();
        let mut b = Claims::default();
        a.claim("/claims_test/x", Direction::ToRos).unwrap();
        let refused = b.claim("/claims_test/x", Direction::FromRos);
        assert!(matches!(
            refused,
            Err(BridgeError::AlreadyBridged { existing: Direction::ToRos, requested: Direction::FromRos, .. })
        ));
        assert!(a.claim("/claims_test/x", Direction::ToRos).is_err(), "twice by the same holder");
        // services are a namespace of their own, as on DDS and on the bus
        b.claim("/claims_test/x", Direction::ServiceFromRos).unwrap();
        assert!(a.claim("/claims_test/x", Direction::ServiceToRos).is_err());
        drop(a);
        b.claim("/claims_test/x", Direction::FromRos).unwrap();
        b.release("/claims_test/x", Direction::FromRos);
        let mut c = Claims::default();
        c.claim("/claims_test/x", Direction::ToRos).unwrap();
    }
}
