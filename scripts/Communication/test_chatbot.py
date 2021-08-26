#! /usr/bin/env python3

from tensorflow.keras.models import load_model
import json
import numpy as np
import random
import pickle
import tensorflow as tf
import nltk
from nltk.stem import WordNetLemmatizer
import time
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('speak',String,queue_size=10)

nltk.download('punkt')
nltk.download('wordnet')


lemmatizer = WordNetLemmatizer()
intents_file = open('intents.json').read()
intents = json.loads(intents_file)
words = pickle.load(open('words.pkl', 'rb'))
classes = pickle.load(open('classes.pkl', 'rb'))

model = load_model('ANCRo_model.h5')

def callback(data):
    messege = (str(data.data))
    print(messege)
    if messege is not '':
        ints = predict_class(messege)
        res = get_response(ints,intents)
        print(res)
        # call(["espeak","-s140 -ven+18 -z",res])
        pub.publish(res)
        

def cleanup_sentance(sentance):
    sentance_words = nltk.word_tokenize(sentance)
    sentance_words = [lemmatizer.lemmatize(word) for word in sentance_words]
    return sentance_words


def bag_of_words(sentance):
    sentance_words = cleanup_sentance(sentance)
    bag = [0] * len(words)
    for w in sentance_words:
        for i, word in enumerate(words):
            if word == w:
                bag[i] = 1
    return np.array(bag)


def predict_class(sentance):
    bow = bag_of_words(sentance)
    res = model.predict(np.array([bow]))[0]
    ERROR_THRESHOLD = 0.15
    results = [[i, r] for i, r in enumerate(res) if r > ERROR_THRESHOLD]
    results.sort(key=lambda x: x[1], reverse=True)

    return_list = []
    for r in results:
        return_list.append({'intent': classes[r[0]], 'probability': str(r[1])})
    return return_list


def get_response(intents_list, intents_jason):
    tag = intents_list[0]['intent']
    list_of_intents = intents_jason['intents']
    for i in list_of_intents:
        if i['tag'] == tag:
            result = random.choice(i['responses'])
            break
    return result

rospy.init_node("Chatbot")


print("I am Ready to talk")

sub = rospy.Subscriber('/phrases',String,callback)
rospy.spin()

# while True:
#   try:
#     print("say")
#     messege = input("")
#     print(messege)
#     ints = predict_class(messege)
#     res = get_response(ints, intents)
#     print(res)
#   except:
#     print("Please Give Some Appropriate Input")
