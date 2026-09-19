// Bindings that `wrangler types` cannot see: secrets (set with `wrangler secret
// put`, or .dev.vars locally) and the test-only migrations binding from
// vitest.config.ts.
interface MowerSecrets {
  PROVISION_TOKEN: string;
  // Cloudflare Realtime TURN key (dashboard -> Realtime -> TURN). Both empty =
  // STUN only: video then works on the LAN but not across networks.
  TURN_KEY_ID?: string;
  TURN_KEY_API_TOKEN?: string;
  // Lifetime of the generated TURN credentials, seconds (default 86400).
  TURN_TTL_S?: string;
}

interface Env extends MowerSecrets {}

declare namespace Cloudflare {
  interface Env extends MowerSecrets {
    TEST_MIGRATIONS: D1Migration[];
  }
}
