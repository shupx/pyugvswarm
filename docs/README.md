# Doc instruction

This doc is built by sphinx in .rst format and uploaded to [readthedocs](https://readthedocs.org/).

Online document: https://pyugvswarm.readthedocs.io/

## Build

Install dependencies:

```bash
pip3 install -U Sphinx
pip3 install sphinx-autobuild
pip3 install sphinx_rtd_theme
pip3 install sphinx_tabs
```
Build:

```bash
make html
```

## view

- #### method1:
  
    Double click `build/html/index.html` and view it in browser.

- #### method2:

    Auto-build by sphinx:
  
  ```bash
  sphinx-autobuild source build/html
  ```

  Then view http://127.0.0.1:8000 in browser. The html will auto update when editing the `source/` files.