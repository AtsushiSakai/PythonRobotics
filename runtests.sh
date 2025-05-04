#!/usr/bin/env bash
cd "$(dirname "$0")" || exit 1
echo "Run test suites! "

# === pytest based test runner ===
# -Werror: warning as error
# --durations=0: show ranking of test durations
# -l (--showlocals); show local variables when test failed
pytest tests -l -Werror --durations=0
