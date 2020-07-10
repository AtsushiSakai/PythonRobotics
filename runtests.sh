#!/usr/bin/env bash
echo "Run test suites! "
export PYTHONWARNINGS=default  # show warning
#python -m unittest discover tests 
#python -Wignore -m unittest discover tests #ignore warning
coverage run -m unittest discover tests # generate coverage file
