import { applyD1Migrations } from "cloudflare:test";
import { env } from "cloudflare:workers";

// Setup files run outside per-test-file storage isolation and may run more
// than once; applyD1Migrations only applies what is missing.
await applyD1Migrations(env.DB, env.TEST_MIGRATIONS);
