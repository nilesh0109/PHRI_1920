#!/usr/bin/env python

from Tkinter import *
import smach
import rospy
import cv2
from PIL import Image, ImageTk
from os.path import dirname, abspath
import os


class VisionFallback(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "confirmed",
            ],
            input_keys=["scene_number", "A_cubes", "B_cubes", "A_image_path", "B_image_path"],
            output_keys=["A_cubes", "B_cubes"],
        )

    def cube_count_gui(self, default_a, default_b, path_a, path_b):
        root = Tk(className="Cube Count")
        w = root.winfo_reqwidth()
        h = root.winfo_reqheight()
        ws = root.winfo_screenwidth()
        hs = root.winfo_screenheight()
        x = (ws / 2) - (w / 2)
        y = (hs / 2) - (h / 2)
        root.geometry('+%d+%d' % (x, y))
        count_a = IntVar()
        count_a.set(default_a)
        count_b = IntVar()
        count_b.set(default_b)
        # label
        label_top = Label(root, text="Select number of cubes for each robot")
        label_top.pack(anchor=CENTER, side=TOP)
        # frame to group left and right frame
        frame_radios=Frame(root)
        frame_radios.pack(anchor=CENTER, side=TOP)
        # left and right frames to group radiobuttons
        frame_left =Frame(frame_radios)
        frame_left.pack(anchor=CENTER, side=LEFT)
        frame_right = Frame(frame_radios)
        frame_right.pack(anchor=CENTER, side=RIGHT)
        #images
        os.system("scp icub@wtmpc29:{} {}/cube_images/".format(path_a, dirname(abspath(__file__))))
        os.system("mv {}/cube_images/{} {}/cube_images/robotA.png".format(dirname(abspath(__file__)), path_a.split("/")[-1], dirname(abspath(__file__))))
        img_a = cv2.imread(dirname(abspath(__file__)) + '/cube_images/robotA.png')
        img_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2RGB)
        img_a = Image.fromarray(img_a)
        img_a = ImageTk.PhotoImage(image=img_a)
        img_label_left = Label(frame_left, image=img_a)
        img_label_left.pack()
        os.system("scp icub@wtmpc29:{} {}/cube_images/".format(path_b, dirname(abspath(__file__))))
        os.system("mv {}/cube_images/{} {}/cube_images/robotB.png".format(dirname(abspath(__file__)), path_b.split("/")[-1], dirname(abspath(__file__))))
        img_b = cv2.imread(dirname(abspath(__file__)) + '/cube_images/robotB.png')
        img_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2RGB)
        img_b = Image.fromarray(img_b)
        img_b = ImageTk.PhotoImage(image=img_b)
        img_label_right = Label(frame_right, image=img_b)
        img_label_right.pack()

        # Labels
        label_left = Label(frame_left, text="Robot A")
        label_left.pack(anchor=W)
        label_right = Label(frame_right, text="Robot B")
        label_right.pack(anchor=W)

        # Radiobuttons left
        rl0 = Radiobutton(frame_left, text="0", variable=count_a, value=0)
        rl0.pack(anchor=W)
        rl1 = Radiobutton(frame_left, text="1", variable=count_a, value=1)
        rl1.pack(anchor=W)
        rl2 = Radiobutton(frame_left, text="2", variable=count_a, value=2)
        rl2.pack(anchor=W)
        rl3 = Radiobutton(frame_left, text="3", variable=count_a, value=3)
        rl3.pack(anchor=W)
        rl4 = Radiobutton(frame_left, text="4", variable=count_a, value=4)
        rl4.pack(anchor=W)
        rl5 = Radiobutton(frame_left, text="5", variable=count_a, value=5)
        rl5.pack(anchor=W)
        rl6 = Radiobutton(frame_left, text="6", variable=count_a, value=6)
        rl6.pack(anchor=W)
        rl7 = Radiobutton(frame_left, text="7", variable=count_a, value=7)
        rl7.pack(anchor=W)
        # Radiobuttons right
        rr0 = Radiobutton(frame_right, text="0", variable=count_b, value=0)
        rr0.pack(anchor=W)
        rr1 = Radiobutton(frame_right, text="1", variable=count_b, value=1)
        rr1.pack(anchor=W)
        rr2 = Radiobutton(frame_right, text="2", variable=count_b, value=2)
        rr2.pack(anchor=W)
        rr3 = Radiobutton(frame_right, text="3", variable=count_b, value=3)
        rr3.pack(anchor=W)
        rr4 = Radiobutton(frame_right, text="4", variable=count_b, value=4)
        rr4.pack(anchor=W)
        rr5 = Radiobutton(frame_right, text="5", variable=count_b, value=5)
        rr5.pack(anchor=W)
        rr6 = Radiobutton(frame_right, text="6", variable=count_b, value=6)
        rr6.pack(anchor=W)
        rr7 = Radiobutton(frame_right, text="7", variable=count_b, value=7)
        rr7.pack(anchor=W)

        button = Button(root, text="Confirm", fg="green", command=root.destroy)
        button.pack(anchor=S, side=BOTTOM)

        root.mainloop()

        return count_a.get(), count_b.get()

    def execute(self, userdata):
        print("\033[92mPlease manually select cube count.\033[0m")

        cubes_a, cubes_b = self.cube_count_gui(userdata.A_cubes, userdata.B_cubes, userdata.A_image_path, userdata.B_image_path)

        userdata.A_cubes = cubes_a
        userdata.B_cubes = cubes_b

        return "confirmed"

if __name__ == "__main__":
    sm = smach.StateMachine(outcomes=['experiment_ended'])

    sm.userdata.scene = "scene_2"
    sm.userdata.A_cubes = 3
    sm.userdata.B_cubes = 4
    sm.userdata.A_image_path = ""
    sm.userdata.B_image_path = ""

    with sm:
        smach.StateMachine.add('FOO', VisionFallback(), transitions={'confirmed': 'experiment_ended'})

    outcome = sm.execute()

    print("A: %i, B: %i" % (sm.userdata.count_a, sm.userdata.count_b))