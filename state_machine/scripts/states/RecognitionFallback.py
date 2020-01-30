#!/usr/bin/env python

from Tkinter import *
import smach
import rospy

class RecognitionFallback(smach.State):

    def sel(self):
       self.question = "{}_question_{}".format(self.scene, self.var.get())


    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "manual_answer",
            ],
            input_keys=["scene"],
            output_keys=["sentence"],
        )

    def mission_protocol(self):
        root = Tk(className="mission protocol")
        w = root.winfo_reqwidth()
        h = root.winfo_reqheight()
        ws = root.winfo_screenwidth()
        hs = root.winfo_screenheight()
        x = (ws / 2) - (w / 2)
        y = (hs / 2) - (h / 2)
        root.geometry('+%d+%d' % (x, y))
        self.var = IntVar()
        R1 = Radiobutton(root, text="Can you tell me more about our mission?", variable=self.var, value=0, command=self.sel)
        R1.pack( anchor = W )

        R2 = Radiobutton(root, text="Have you been to Xantonia before?", variable=self.var, value=1, command=self.sel)
        R2.pack( anchor = W )

        R3 = Radiobutton(root, text="What type of ship is this?", variable=self.var, value=2, command=self.sel)
        R3.pack( anchor = W)

        R4 = Radiobutton(root, text="What is the crews function on the ship?", variable=self.var, value=3, command=self.sel)
        R4.pack( anchor = W)

        button = Button(root, text="Confirm", fg="green", command=root.destroy)
        button.pack(side=BOTTOM)

        root.mainloop()

    def emergency_protocol(self):
        root = Tk(className="emergency protocol")
        w = root.winfo_reqwidth()
        h = root.winfo_reqheight()
        ws = root.winfo_screenwidth()
        hs = root.winfo_screenheight()
        x = (ws / 2) - (w / 2)
        y = (hs / 2) - (h / 2)
        root.geometry('+%d+%d' % (x, y))
        self.var = IntVar()
        R1 = Radiobutton(root, text="Lieutenants, can you tell me more about your suggestions?", variable=self.var, value=0, command=self.sel)
        R1.pack( anchor = W )

        R2 = Radiobutton(root, text="How confident are you in your solutions?", variable=self.var, value=1, command=self.sel)
        R2.pack( anchor = W )

        R3 = Radiobutton(root, text="What do you think about each others ideas?", variable=self.var, value=2, command=self.sel)
        R3.pack( anchor = W)

        button = Button(root, text="Confirm", fg="green", command=root.destroy)
        button.pack(side=BOTTOM)

        root.mainloop()

    def done_confirmation(self):
        self.question = "done_confirmation"
        root = Tk(className="confirm")
        root.geometry("200x50")
        w = root.winfo_reqwidth()
        h = root.winfo_reqheight()
        ws = root.winfo_screenwidth()
        hs = root.winfo_screenheight()
        x = (ws / 2) - (w / 2)
        y = (hs / 2) - (h / 2)
        root.geometry('+%d+%d' % (x, y))
        self.var = IntVar()

        label = Label(root, text="Wendigo, I'm done!")
        label.pack()

        button = Button(root, text="Continue", fg="green", command=root.destroy)
        button.pack(side=BOTTOM)

        root.mainloop()

    def execute(self, userdata):
        print("\033[91mSpeech recognition failed, please manually select participant input.\033[0m")

        self.scene = userdata.scene

        self.question = self.scene + "_question_0"

        if self.scene == "done":
            self.done_confirmation()
        elif self.scene in ("scene_0", "scene_1"):
            self.mission_protocol()
        else:
            self.emergency_protocol()

        userdata.sentence = self.question

        return "manual_answer"


class BAR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        # Your state execution goes here
        raw_input("Press enter to continue")
        return 'outcome1'

if __name__ == "__main__":
    sm = smach.StateMachine(outcomes=['experiment_ended'])

    sm.userdata.scene = "scene_2"
    sm.userdata.sentence = ""

    with sm:
        smach.StateMachine.add('FOO', RecognitionFallback(), transitions={'manual_answer': 'BAR'})
        smach.StateMachine.add('BAR', BAR(), transitions={'outcome1': 'experiment_ended'})

    outcome = sm.execute()

    print(sm.userdata.sentence)