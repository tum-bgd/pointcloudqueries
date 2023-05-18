py -3.7  -m pip install wheel
py -3.8  -m pip install wheel
py -3.9  -m pip install wheel
py -3.10 -m pip install wheel
py -3.11 -m pip install wheel


py -3.7 setup.py bdist_wheel
py -3.8 setup.py bdist_wheel
py -3.9 setup.py bdist_wheel
py -3.10 setup.py bdist_wheel
py -3.11 setup.py bdist_wheel
dir dist
