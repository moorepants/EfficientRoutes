This package provides a tool to compare bicycle routes on an energy and time
consumption basis.

Dependencies
============

virtualenv
----------

Virtualenv is not a dependency, but very useful. Create a new environment with::

   virtualenv efficientroutes

numpy
-----

pip install numpy

matplotlib
----------

pip install matplotlib

python-sundials
---------------

python-sundials was a bit of a pain to install on my Ubuntu 12.04 machine. I
think it was primarily due to me trying to use the latest versions of Cython
(0.16) and Sundials (2.5.0).  Once I reverted to older versions of those, it
installed correctly.

Activate the virtualenv for the project::

   source efficientroutes/bin/activate

Install Cython 0.14.1::

   pip install Cython==0.14.1

Download python-sundials 0.5 and extract into ``python-sundials``

The ``python-sundials/setup.py`` file needs to be cleaned up before it will run
cross platform. Comment out or remove the sys.argv.append lines in
``python-sundials/setup.py``

Download and build sundials 2.4.0 from

https://computation.llnl.gov/casc/sundials/main.html

Extract the tarball::

   tar -xzf sundials-2.4.0.tar.gz
   cd sundials-2.4.0/
   ./configure --with-pic # the --with-pic argument is necessary
   make

Copy the built sundials files into the python-sundials directory::

   find . -iname "*.a" -exec cp {} ../python-sundials/sundials/ \;
   cp -r include ../python-sundials/sundials/

Now install and test python-sundials::

   cd ../python-sundials
   python setup.py install
   cd ..
   python python-sundials/sundials/test_cv_simple.py

Run the test problems in python-sundials/sundials to see if it works.
