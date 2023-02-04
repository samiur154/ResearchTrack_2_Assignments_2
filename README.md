[Research Track 2_Assignment 2](https://corsi.unige.it/off.f/2021/ins/51207) <br>
Course Instructor: [Prof. CARMINE RECCHIUTO](https://rubrica.unige.it/personale/UkNDWV1r) <br>

# Docygen Documentation for Robot Control 

## Abstract ##
Documents describing the software system, including technical design documents, software requirements, and UML diagrams. This documentation that’s partly auto-generated from docstrings in [my code](https://github.com/samiur154/ResearchTrack_2_Assignments_2.git). This documentation is developed for the the project [Robot Control](https://github.com/samiur154/ResearchTrack_2_Assignments_2.git). For this, ***Doxygen*** tool is used. 

## Introduction ##
In this documentation, [Doxygen](https://www.doxygen.nl/manual/docblocks.html) tool is used. *Doxygen* was originally created for **Cpp**, but it has now facilities for the documentation of software projects in a range of languages. 

Doxygen is the de facto standard tool for generating documentation from annotated C++ sources, but it also supports other popular programming languages such as C, Objective-C, C#, PHP, Java, Python, IDL (Corba, Microsoft, and UNO/OpenOffice flavors), Fortran, VHDL.

* It can generate an on-line documentation browser (in HTML) and/or an off-line reference manual (in LaTeX) from a set of documented source files. There is also support for generating output in RTF (MS-Word), PostScript, hyperlinked PDF, compressed HTML, and Unix man pages. The documentation is extracted directly from the sources, which makes it much easier to keep the documentation consistent with the source code.
* You can configure doxygen to extract the code structure from undocumented source files. This is very useful to quickly find your way in large source distributions. Doxygen can also visualize the relations between the various elements by means of include dependency graphs, inheritance diagrams, and collaboration diagrams, which are all generated automatically.
* You can also use doxygen for creating normal documentation

## Installation ##

```
sudo apt-get install –y doxygen
sudo apt-get install doxygen-gui
```

#On Ubuntu (and on the Docker image, but without sudo).

Now, ***Doxygen GUI*** will open as shown in figure below:

<p align="center">
  <img width="800" height="500" src="https://github.com/samiur154/ResearchTrack_2_Assignments_2/blob/e0e59417cac5a8f9791dc9ef594090a206cbf07a/doxyGUI.png">
</p>

Now choose the ***working directory from which doxygen will run*** in *Step 1* as shown in figure above. After choosing the folder path, Provide some information about the project you are documenting such as: **Project Name, Project synopsis, Project version or id, Project logo** (if any), and so on. After giving the information , now ***specify the directory to scan for source code*** in *Step 2* which means add the source code directory. Mark the box for ***scan recursively***. After this, ***specify the directory where doxygen should put the generated doucmentation*** i.e. ***Destination directory*** as shown in the figure above. In our case, desination directory is *_build*. Save this as ***Doxygen***.
