# ROS-Dash Video Stream

An experimental video streaming demo for publishing videos over ROS, listening, and finally, streaming the content to a webpage running Flask/Plotly-Dash.

You must have ROS installed on your machine. Note that we recently tested the scripts on python2.7, but early versions also worked with python3 + ROS (using workarounds)



To setup the requirements:
```
pip install -r requirements.txt
```

To run, set the `MODE` in `main.py` to your preferred video reading mode (available options are in the code comments)
and eventually:

```
python main.py
```
