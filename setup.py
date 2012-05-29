from setuptools import setup

setup(
    name='EfficientRoutes',
    version='0.1.0dev',
    author=['Jason Keith Moore', 'Mont Hubbard'],
    author_email=['moorepants@gmail.com', 'mhubbard@ucdavis.edu'],
    packages=['efficientroutes', 'efficientroutes.test'],
    url='http://github.com/moorepants/EfficientRoutes',
    license='LICENSE.txt',
    description='Compares bicycle routes based on energy and time consumption.',
    long_description=open('README.rst').read(),
    classifiers=['Programming Language :: Python',
                 'Programming Language :: Python :: 2.7',
                 'Operating System :: OS Independent',
                 'Development Status :: 4 - Beta',
                 'Intended Audience :: Science/Research',
                 'License :: OSI Approved :: BSD License',
                 'Natural Language :: English',
                 'Topic :: Scientific/Engineering',
                 'Topic :: Scientific/Engineering :: Physics']
)
