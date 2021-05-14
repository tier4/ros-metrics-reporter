#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path


plotly_inplace = '{{< load-plotly >}}\n\
{{< plotly json="/plotly/__PLOTLY_FIGURE_NAME__.json" height="400px" >}}\
'

def replace_token(file: Path, package: str):
  # Read file, replace token and overwrite file
  with open(file) as f:
    text_lines = f.read()

  text_lines = text_lines.replace('__TEMPLATE__', package)
  text_lines = text_lines.replace('__PLOTLY_FIGURE__', plotly_inplace)
  text_lines = text_lines.replace('__PLOTLY_FIGURE_NAME__', package + '/CCN_violate')

  with open(file, 'w') as f:
    f.write(text_lines)

def copy_template(src: str, dest: str, packages: list):
  # Copy all files from template/hugo/content/ to hugo content directory
  markdown_dir_src = Path(src, 'content')
  markdown_dir_dest = Path(dest, 'content')
  dir_util.copy_tree(markdown_dir_src, str(markdown_dir_dest))
  template = Path(dest, 'content', 'packages', 'TEMPLATE.md')

  for package in packages:
    filename = dest / 'content' / 'packages' / (package + '.md')
    shutil.copy(template, filename)
    # Peplace token
    replace_token(filename, package)

  template.unlink()

  # Copy layouts to hugo layouts directory
  layout_dir_src = Path(src, 'layouts', 'shortcodes')
  layout_dir_dest = Path(dest, 'layouts', 'shortcodes')
  dir_util.copy_tree(layout_dir_src, str(layout_dir_dest))
