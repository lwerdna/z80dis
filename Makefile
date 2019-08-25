CPPFLAGS = 
CPPFLAGS += -I/usr/local/include
CPPFLAGS += -I/$(HOME)/Downloads/z80ex/include
CPPFLAGS += -std=c++11 -O0 -g

.PHONY: package pypitest

all: gofer.so

gofer.so: gofer.cpp
	g++ $(CPPFLAGS) \
		-lbfd -liberty -lopcodes -lz \
		-shared -o gofer.so gofer.cpp \
		$(HOME)/Downloads/z80ex/z80ex_dasm.o \
		-Wl,-headerpad_max_install_names

package:
	python3 setup.py sdist bdist_wheel

install:
	pip install ./dist/*.whl

pypitest:
	python3 -m twine upload --repository-url https://test.pypi.org/legacy/ dist/*
	# https://test.pypi.org/project/z80dis
	# pip install --index-url https://test.pypi.org/simple/ --no-deps z80dis

pypi:
	python3 -m twine upload dist/*
	# https://pypi.org/project/z80dis/
	# pip install z80dis

clean:
	rm -f gofer.so
	rm -rf gofer.so.dSYM
	rm -rf dist build __pycache__ z80dis.egg-info
