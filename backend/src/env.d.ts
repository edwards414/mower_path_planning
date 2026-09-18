// Bindings that `wrangler types` cannot see: secrets (set with `wrangler secret
// put`, or .dev.vars locally) and the test-only migrations binding from
// vitest.config.ts.
declare namespace Cloudflare {
  interface Env {
    PROVISION_TOKEN: string;
    TEST_MIGRATIONS: D1Migration[];
  }
}
