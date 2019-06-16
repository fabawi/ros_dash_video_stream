import dash_app

from threading import Thread

import video_generators
from video_streamers import CV2VideoStreamer, ROSVideoStreamer
from ros_video_listener import ROSVideoListener

# set the mode to one of the available templates for quick testing
MODE = 'ROS_LISTENER' # 'CV2_CAM', 'CV2_WEB_VIDEO', 'CV2_LOCAL_VIDEO', 'CV2_NOISE_SYNTHETIC', 'CV2_TIMESTAMP_SYNTHETIC', 'ROS_LISTENER'


def run_webapp():
    dash_app.app.run_server(host='127.0.0.1', port=8050, debug=True, use_reloader=False)


if __name__ == "__main__":

    if MODE == 'CV2_CAM':
        dash_app.streamer = CV2VideoStreamer(0)
        wa = Thread(target=run_webapp)
        wa.start()
    elif MODE == 'CV2_WEB_VIDEO':
        dash_app.streamer = CV2VideoStreamer('rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov')
        wa = Thread(target=run_webapp)
        wa.start()
    elif MODE == 'CV2_LOCAL_VIDEO':
        dash_app.streamer = CV2VideoStreamer('samples/moon.avi')
        wa = Thread(target=run_webapp)
        wa.start()
    elif MODE == 'CV2_NOISE_SYNTHETIC':
        stream = video_generators.VideoSource(image_size=(1080, 1920))
        dash_app.streamer = CV2VideoStreamer(stream)
        wa = Thread(target=run_webapp)
        wa.start()
    elif MODE == 'CV2_TIMESTAMP_SYNTHETIC':
        stream = video_generators.TimeStampVideo(image_size=(1080, 1920), time_fmt ='%H:%M:%S.%f')
        dash_app.streamer = CV2VideoStreamer(stream)
        wa = Thread(target=run_webapp)
        wa.start()
    elif MODE == 'ROS_LISTENER':
        # start ros_video_publisher.py before running the ROS listener node. Make sure roscore is running as well
        dash_app.streamer = ROSVideoStreamer()
        wa = Thread(target=run_webapp)
        wa.start()
        rl = ROSVideoListener(visualize=False)
        rl.subscribe(dash_app.streamer)