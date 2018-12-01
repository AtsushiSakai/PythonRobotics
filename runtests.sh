echo "Run test suites! "
#python -m unittest discover tests 
#python -Wignore -m unittest discover tests #ignore warning
coverage run -m unittest discover tests # generate coverage file
