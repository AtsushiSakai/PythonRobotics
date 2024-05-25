## Python Robotics Documentation

This folder contains documentation for the Python Robotics project.


### Build the Documentation

#### Install Sphinx and Theme

```
pip install sphinx sphinx-autobuild sphinx-rtd-theme sphinx_rtd_dark_mode sphinx_copybutton sphinx_rtd_dark_mode
```

#### Building the Docs

In the `docs/` folder:
```
make html
```

if you want to building each time a file is changed:

```
sphinx-autobuild . _build/html
```

#### Check the generated doc

Open the index.html file under docs/_build/
