[Research Track 2_Assignment 2](https://corsi.unige.it/off.f/2021/ins/51207) <br>
Course Instructor: [Prof. CARMINE RECCHIUTO](https://rubrica.unige.it/personale/UkNDWV1r) <br>

# Sphinx Documentation for Robot Control 

## Abstract ##
Documents describing the software system, including technical design documents, software requirements, and UML diagrams. This documentation that’s partly auto-generated from docstrings in [my code](https://github.com/samiur154/ResearchTrack_2_Assignments_2.git). This documentation is developed for the the project [Robot Control](https://github.com/samiur154/ResearchTrack_2_Assignments_2.git). For this, ***Sphinx*** tool is used. 

## Introduction ##
In this documentation, [Sphinx](https://www.sphinx-doc.org/en/master/) tool is used. *Sphinx* was originally created for **Python**, but it has now facilities for the documentation of software projects in a range of languages. Sphinx uses [reStructuredText](https://docutils.sourceforge.io/rst.html) as its markup language.

Here are some of Sphinx’s major features:

* **Output formats:** HTML (including Windows HTML Help), LaTeX (for printable PDF versions), ePub, Texinfo, manual pages, plain text.
* **Extensive cross-references:** semantic markup and automatic links for functions, classes, citations, glossary terms and similar pieces of information.
* **Hierarchical structure:** easy definition of a document tree, with automatic links to siblings, parents and children.
* **Automatic indices:** general index as well as a language-specific module indices.
* **Code handling:** automatic highlighting using the Pygments highlighter.
* **Extensions:** automatic testing of code snippets, inclusion of docstrings from Python modules (API docs) via built-in extensions, and much more functionality via third-party extensions.
* **Themes:** modify the look and feel of outputs via creating themes, and re-use many third-party themes.
* **Contributed extensions:** dozens of extensions contributed by users; most of them installable from PyPI.

## Installation ##

To create documentation, *Sphinx* tool is required on your system. To install Sphinx follow the steps given below:

```
sudo apt-get install python3-sphinx
```
*For Docker user execute the aforementioned command without sudo*

```
pip3 install breathe
```
```
pip3 install sphinx-rtd-theme
```
With these three commands, you are installing spinhx, plus breathe, which is a tool for integrating doxygen documentation in sphinx, and thus comment also cpp code, and the ReadTheDocs theme, one of the most used theme for creating documentation.

After installation of *Sphinx*, follow the steps given below:

**Step 1:** 
```
sphinx-quickstart
```
After running the command, accept the defaults. It’ll look something like this: 

<p align="center">
  <img width="1000" height="260" src="https://github.com/samiur154/ResearchTrack_2_Assignments_2/blob/c09d92b07536971eb3f9cb57c6f8c37cf84683e7/s1.png">
</p>

Figure above shows that *Separate source and build directories (y/n) [n]: n*. 

**Step 2:** Now, press entre write the ***Project Name, Author Name, Project Release***. as shown in figure below.

<p align="center">
  <img width="1000" height="260" src="https://github.com/samiur154/ResearchTrack_2_Assignments_2/blob/c09d92b07536971eb3f9cb57c6f8c37cf84683e7/s2.png">
</p>

**Step 3:** After entering the required details, now choose the language in which documentation to be prepared. 

<p align="center">
  <img width="1000" height="260" src="https://github.com/samiur154/ResearchTrack_2_Assignments_2/blob/c09d92b07536971eb3f9cb57c6f8c37cf84683e7/s3.png">
</p>

Figure above shows the language choosen for Documentation.
