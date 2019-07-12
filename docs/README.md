## Python Robotics Documentation

This folder contains documentation for the Python Robotics project.


### Build the Documentation

#### Install Sphinx and Theme

```
pip install sphinx sphinx-autobuild sphinx-rtd-theme
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

### Jupyter notebook integration

When you want to generate rst files from each jupyter notebooks,

you can use

```
cd path/to/docs
python jupyternotebook2rst.py
```

