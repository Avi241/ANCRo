#! /usr/bin/env python3
import argparse
import os
import struct
from datetime import datetime
from threading import Thread

import numpy as np
import pvporcupine
import pyaudio
import soundfile


import rospy
from std_msgs.msg import String

rospy.init_node("wake_publisher")
pub = rospy.Publisher('wake',String,queue_size=10)

rate = rospy.Rate(2)
msg_str = String()
msg_str = ""

def trig():
    pub.publish(msg_str)
    print("pub")


class PorcupineDemo(Thread):
    """
    Microphone Demo for Porcupine wake word engine. It creates an input audio stream from a microphone, monitors it, and
    upon detecting the specified wake word(s) prints the detection time and wake word on console. It optionally saves
    the recorded audio into a file for further debugging.
    """

    def __init__(
        self,
        library_path,
        model_path,
        keyword_paths,
        sensitivities,
        input_device_index=None,
        output_path=None,
    ):

        """
        Constructor.
        :param library_path: Absolute path to Porcupine's dynamic library.
        :param model_path: Absolute path to the file containing model parameters.
        :param keyword_paths: Absolute paths to keyword model files.
        :param sensitivities: Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A
        higher sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 will
        be used.
        :param input_device_index: Optional argument. If provided, audio is recorded from this input device. Otherwise,
        the default audio input device is used.
        :param output_path: If provided recorded audio will be stored in this location at the end of the run.
        """

        super(PorcupineDemo, self).__init__()

        self._library_path = library_path
        self._model_path = model_path
        self._keyword_paths = keyword_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index

        self._output_path = output_path
        if self._output_path is not None:
            self._recorded_frames = []

    def run(self, _callback=None):
        """
        Creates an input audio stream, instantiates an instance of Porcupine object, and monitors the audio stream for
        occurrences of the wake word(s). It prints the time of detection for each occurrence and the wake word.
        """

        keywords = list()
        for x in self._keyword_paths:
            keywords.append(os.path.basename(x).replace(".ppn", "").split("_")[0])

        porcupine = None
        pa = None
        audio_stream = None
        try:
            porcupine = pvporcupine.create(
                library_path=self._library_path,
                model_path=self._model_path,
                keyword_paths=self._keyword_paths,
                sensitivities=self._sensitivities,
            )

            pa = pyaudio.PyAudio()

            audio_stream = pa.open(
                rate=porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=porcupine.frame_length,
                input_device_index=self._input_device_index,
            )

            print("Listening {")
            for keyword, sensitivity in zip(keywords, self._sensitivities):
                print("  %s (%.2f)" % (keyword, sensitivity))
            print("}")

            while True:
                pcm = audio_stream.read(porcupine.frame_length)
                pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

                if self._output_path is not None:
                    self._recorded_frames.append(pcm)

                result = porcupine.process(pcm)
                if result >= 0:
                    print("[%s] Detected %s" % (str(datetime.now()), keywords[result]))
                    if _callback:
                        _callback()

        except KeyboardInterrupt:
            print("Stopping ...")
        finally:
            if porcupine is not None:
                porcupine.delete()

            if audio_stream is not None:
                audio_stream.close()

            if pa is not None:
                pa.terminate()

            if self._output_path is not None and len(self._recorded_frames) > 0:
                recorded_audio = np.concatenate(self._recorded_frames, axis=0).astype(
                    np.int16
                )
                soundfile.write(
                    self._output_path,
                    recorded_audio,
                    samplerate=porcupine.sample_rate,
                    subtype="PCM_16",
                )

    @classmethod
    def show_audio_devices(cls):
        fields = ("index", "name", "defaultSampleRate", "maxInputChannels")

        pa = pyaudio.PyAudio()

        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            print(", ".join("'%s': '%s'" % (k, str(info[k])) for k in fields))

        pa.terminate()


def main():
    keyword_paths = [pvporcupine.KEYWORD_PATHS[x] for x in ["blueberry"]]
    p = PorcupineDemo(
        library_path=pvporcupine.LIBRARY_PATH,
        model_path=pvporcupine.MODEL_PATH,
        keyword_paths=keyword_paths,
        sensitivities=([0.5] * len(keyword_paths)),
        output_path=None,
        input_device_index=None,
    )
    p.run(trig)


if __name__ == "__main__":
    main()
