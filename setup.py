from setuptools import setup, find_packages

setup(name='pomdp-py',
      packages=find_packages(),
      version='1.0.0',
      description='Python POMDP Library.',
      install_requires=[
          'numpy',
          'scipy',
          'tqdm',
          'matplotlib',
          'pygame',        # for some tests
          'opencv-python',  # for some tests
      ],
      license="None",
      author='Edward Kim',
      author_email='edward.kim@anu.edu.au',
      keywords = ['Partially Observable Markov Decision Process', 'POMDP'],
    #   package_data={"pomdp_py": ["*.pxd", "*.pyx"],
    #                 "pomdp_problems": ["*.pxd", "*.pyx"]},
      zip_safe=False
)
