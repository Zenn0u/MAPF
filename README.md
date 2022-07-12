# MAPF
Multi Agent Path Finding
# 0 Task 0: Preparing for the Project
## 0.1 Installing Python 3
This project requires a Python 3 installation with the numpy and matplotlib packages. On Ubuntu Linux, download python by using:

```sudo apt install python3 python3-numpy python3-matplotlib```

On Mac OS X, download Anaconda 2019.03 with Python 3.7 from https://www.anaconda.com/distribution/#download-section and follow the installer. You can verify your installation by
using:

```python3 --version```

It may be necessary to use "pythonw" instead depending on the particular version of Mac OS X.
On Windows, download Anaconda 2019.03 with Python 3.7 from https://www.anaconda.com/
distribution/#download-section.
On Ubuntu Linux and Mac OS X, use python3 to run python. On Windows, use python instead.
You can use a plain text editor for the project. If you would like to use an IDE, we recommend that
you download PyCharm from https://www.jetbrains.com/pycharm/. The free community edition
succesfully, but you can get the professional edition for free as well, see https://www.jetbrains.
com/student/ for details.

## 0.2 Installing the MAPF Software
Download the archive with the provided MAPF software and extract it on your computer.

## 0.3 Learning about MAPF
Read the provided textbook-style overview of MAPF.

## 0.4 Understanding Independent Planning
Execute the independent MAPF solver by using:

```python run_experiments.py --instance instances/exp0.txt --solver Independent```

If you are successful, you should see an animation:
![image](https://user-images.githubusercontent.com/39423448/178403151-861b618d-bc9f-49fa-91a7-a2e3efeb53a4.png)
![image](https://user-images.githubusercontent.com/39423448/178403166-391badd3-c8b2-4971-97c4-91b62d6a1ad4.png)
![image](https://user-images.githubusercontent.com/39423448/178403175-67626882-bea3-4126-9895-aceb70d29a41.png)

The independent MAPF solver plans for all agents independently.py Their paths do not collide with
the environment but are allowed to collide with the paths of the other agents. Thus, there is a
collision when the blue agent 1 stays at its goal cell while the green agent 0 moves on top of it. In
your animation, both agents turn red when this happens, and a warning is printed on the terminal
notifying you about the details of the collision.
Try to understand the independent MAPF solver in independent.py. The first part defines the
class IndependentSolver and its constructor:
