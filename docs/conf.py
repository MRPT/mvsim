# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

import sphinx_rtd_theme
import subprocess
import os
import datetime

# read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'
# if read_the_docs_build:

# Build doxygen docs too:
subprocess.call('mkdir -p _build/html/doxygen/; doxygen', shell=True)


# -- Project information -----------------------------------------------------

project = 'mvsim'
author = u'Jose Luis Blanco Claraco'
copyright = datetime.date.today().strftime("%Y") + u', ' + author

html_theme_options = {
    # 'canonical_url': '',
    'analytics_id': 'UA-21128561-9',  # Provided by Google in your dashboard
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # 'style_nav_header_background': 'white',
    # Toc options
    # 'collapse_navigation': False,
    # 'sticky_navigation': False,
    # 'navigation_depth': 4,
    # 'includehidden': True,
}

html_theme = "sphinx_rtd_theme"

# -- General configuration ---------------------------------------------------

# To tell readthedocs.org to use index.rst as main doc:
master_doc = 'index'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx_rtd_theme',
    'sphinx.ext.mathjax',
    'sphinx_design',
    'sphinx.ext.autosectionlabel',
    'sphinxcontrib.bibtex',
]

bibtex_bibfiles = ['refs.bib']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_path = ["_themes", ]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_build/html/doxygen/"]  # '_static'


rst_epilog = """
.. role:: raw-html(raw)
    :format: html
"""
