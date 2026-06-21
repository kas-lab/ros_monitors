# Configuration file for the Sphinx documentation builder.
#
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# Prefer source tree over any installed workspace copy.
_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_repo, 'ros_monitors_functions'))
sys.path.insert(0, os.path.join(_repo, 'ros_monitors_samples'))

project = 'ros_monitors'
copyright = '2025, KAS-Lab'
author = 'Gustavo Rezende Silva'
release = '0.0.0'

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "myst_parser",
]

autosummary_generate = True

# tf_transformations uses np.float which was removed in NumPy 1.24
autodoc_mock_imports = ["tf_transformations"]

templates_path = ["_templates"]

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

myst_heading_anchors = 3
