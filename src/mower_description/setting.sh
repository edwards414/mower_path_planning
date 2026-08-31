export ONSHAPE_API=https://cad.onshape.com
# Source this file after exporting the Onshape credentials (or after sourcing
# the gitignored sibling file setting.local.sh). The tracked loader only
# validates that credentials exist; it never stores them.

: "${ONSHAPE_ACCESS_KEY:?Set ONSHAPE_ACCESS_KEY in the environment or setting.local.sh}"
: "${ONSHAPE_SECRET_KEY:?Set ONSHAPE_SECRET_KEY in the environment or setting.local.sh}"
