#!/usr/bin/env python

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

from flask import Flask, Response

from video_streamers import VideoStreamer

server = Flask(__name__)
app = dash.Dash(__name__, server=server)
streamer = VideoStreamer()  # must be defined in main.py


def gen(resource):
    while True:
        frame = resource.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.server.before_first_request
def initialization():
    global video_source

    video_source = streamer


@server.route('/video_feed')
def video_feed():
    return Response(gen(video_source),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

app.layout = html.Div([
    html.H1("ROS Video Test"),
    html.Img(src="/video_feed"),
    dcc.Interval(
            id='interval-component',
            interval=1*1000, # in milliseconds
            n_intervals=0
        ),
    html.H5('FPS: 0', id='fps-label')
])


@app.callback(Output('fps-label', 'children'),
              [Input('interval-component', 'n_intervals')])
def update_fps(n):
    return 'FPS: ' + str(video_source.get_fps())


if __name__ == '__main__':
    app.run_server(host='127.0.0.1', port=8050, debug=True, use_reloader=False)