#! /usr/bin/python
# encoding: utf-8

from gi.repository import Gtk

import rospy
from std_msgs.msg import String

TEXT1 = "\\vol=100\\Salut Diego! \\vct=120\\ \\emph=1\\Regarde\\vct=100\\, j'ai reçu une lettre de Mimi ! \\pau=500\\ Est-ce que tu veux me la lire ?"
TEXT2 = "\\vol=100\\Tu as vu la photo ?"
TEXT3 = "\\vol=100\\Et \\vct=110\\ \\emph=2\\ça ? \\pau=500\\ \\vct=100\\ qu'est-ce que c'est ?"
TEXT4 = "\\vol=100\\Bon, j'ai une idée de ce qu'on va écrire à Mimi ! Tu peux l'écrire s'il te plait ?"
TEXT5 = "\\vol=100\\Et pour l'objet, on dit quoi ?"
TEXT6 = "\\vol=100\\Allez, à moi. Est-ce que tu peux me montrer ton modèle ?"
TEXT7 = "\\vol=100\\\\vct=90\\Je n'y arriverai \\emph=2\\jamais!\\pau=500\\ Je n'arrive même pas à faire \\emph=2\\ça !"
TEXT8 = "\\vol=100\\Bon la semaine prochaine, tu me montres des mots, d'accord ?"

pub_speech = rospy.Publisher('speech', String, queue_size=10)
pub_write = rospy.Publisher('words_to_write', String, queue_size=10)
rospy.init_node('interaction_ui', anonymous=True)

class InteractionUI:

    def __init__(self):
        self.builder = Gtk.Builder()
        self.builder.add_from_file('diego.glade')

        self.builder.connect_signals(self)

        self.textbuffer = self.builder.get_object("text_to_send").get_buffer()
        window = self.builder.get_object("window1")
        window.show_all()

    def onDeleteWindow(self, *args):
        Gtk.main_quit(*args)

    def text1(self, button):
        pub_speech.publish(TEXT1)
    def text2(self, button):
        pub_speech.publish(TEXT2)
    def text3(self, button):
        pub_speech.publish(TEXT3)
    def text4(self, button):
        pub_speech.publish(TEXT4)
    def text5(self, button):
        pub_speech.publish(TEXT5)
    def text6(self, button):
        pub_speech.publish(TEXT6)
    def text7(self, button):
        pub_speech.publish(TEXT7)
    def text8(self, button):
        pub_speech.publish(TEXT8)


    def onSend(self, button):
        start_iter = self.textbuffer.get_start_iter()
        end_iter = self.textbuffer.get_end_iter()
        text = self.textbuffer.get_text(start_iter, end_iter, True) 
        print(text)
        pub_speech.publish(text)

    def onWrite(self, button):
        text = self.builder.get_object("text_to_write").get_text()
        print(text)
        pub_write.publish(text)



if __name__ == "__main__":
    InteractionUI()
    Gtk.main()
