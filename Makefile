all:
	pip uninstall -y pointcloudqueries
	python3 setup.py bdist_wheel
	pip install dist/*
