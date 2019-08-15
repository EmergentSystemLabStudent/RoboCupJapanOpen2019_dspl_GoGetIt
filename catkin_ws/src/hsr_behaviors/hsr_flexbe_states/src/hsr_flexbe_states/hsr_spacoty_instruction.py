#!/usr/bin/env python
from flexbe_core import EventState, Logger

class hsr_SpaCoTyInstruction(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- save_folder    string    save the dataset to save_folder directory(default:default)

    ># sentence        string    recognition

    #> move_name       string    "None(default:tidy up)"

    #> save_folder     string    default: "default"

    <= datadata        getting data

    <= training        training

    <= tidy_nearly     tidy_nearly

    <= tidy_greedy     tidy_greedy

    <= failed          miss commad

    <= end             finish
    '''

    def __init__(self, save_folder="default"):
        super(hsr_SpaCoTyInstruction,self).__init__(outcomes=['datadata','training','tidy_baseline_nearly','tidy_baseline_randomly','tidy_greedy','failed','end'],input_keys=['sentence'],output_keys=['move_name','save_folder'])
        self.sentence = ""
        self._save_name = save_folder

    def execute(self, userdata):
        userdata.save_folder = self._save_name
        if "get dataset" in self.sentence:
            return 'datadata'
        elif "learn spatial concept" in self.sentence:
            return 'training'
        elif "tidy up randomly" in self.sentence:
            #userdata.move_name = "tidy up"
            return 'tidy_baseline_randomly'
        elif "tidy up nearly" in self.sentence:
            #userdata.move_name = "tidy up"
            return 'tidy_baseline_nearly'
        elif "tidy up greedy" or "greedy" in self.sentence:
            return 'tidy_greedy'
        elif "byebye" in self.sentence:
            return 'end'
        else:
            return 'failed'


    def on_enter(self, userdata):
        self.sentence = userdata.sentence
        self.sentence = self.sentence.replace(".","")
        self.sentence = self.sentence.replace("-"," ")
        self.sentence = self.sentence.replace("  "," ")
        self.sentence = self.sentence.replace(" the "," ")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
