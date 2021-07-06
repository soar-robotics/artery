#!/usr/bin/env python

# reference variable to Storyboard Omnet++ module: board
import storyboard
import timeline

print ("demo.py successfully imported...")

def createStories(board):
	# condition triggering after 10 simulated seconds
	timeCondition = storyboard.TimeCondition(timeline.seconds(10))

	# select police car
	carSetCondition = storyboard.CarSetCondition("evw_veh")

	# create signal effect
	signalEffect = storyboard.SignalEffect("EVW")

	# combine conditions
	condition = storyboard.AndCondition(timeCondition, carSetCondition)

	# create story by linking effect and conditions together
	story = storyboard.Story(condition, [signalEffect])

	# activate story
	board.registerStory(story)

	#RWW
	carSetConditionRWW = storyboard.CarSetCondition("rww_veh")

	# create signal effect
	signalEffect = storyboard.SignalEffect("RWW")

	storyRWW = storyboard.Story(carSetConditionRWW, [signalEffect])

	board.registerStory(storyRWW)

	#EEBL
	cond1 = storyboard.TimeCondition(timeline.milliseconds(15000))

	cond2 = storyboard.CarSetCondition({"eebl_veh"})
	effect0 = storyboard.SpeedEffect(1.0)
	#and0 = storyboard.AndCondition(cond0, cond1)
	and1 = storyboard.AndCondition(cond1, cond2)
	cond3 = storyboard.TimeCondition(timeline.seconds(40))
	or0 = storyboard.OrCondition(cond3, and1)
	story1 = storyboard.Story(or0, [effect0])

	# Create Story
	story = storyboard.Story(or0, [effect0])
	cond4 = storyboard.TimeCondition(timeline.seconds(80), timeline.seconds(90))
	cond5 = storyboard.CarSetCondition({"flow0.1"})
	and2 = storyboard.AndCondition(cond4, cond5)
	effect1 = storyboard.SpeedEffect(0.44)
	story2= storyboard.Story(and2, [effect1])
	#board.registerStory(story1)
	board.registerStory(story2)
	'''
	# Create coordinates needed for the PolygonCondition
	coord0 = storyboard.Coord(0.0, 0.0)
	coord1 = storyboard.Coord(3000.0, 0.0)
	coord2 = storyboard.Coord(3000.0, 1600.0)
	coord3 = storyboard.Coord(0.0, 1600.0)

	# Create PolygonCondition
	cond0 = storyboard.PolygonCondition([coord0, coord1, coord2, coord3])

	# Create TimeCondition
	cond1 = storyboard.TimeCondition(timeline.milliseconds(15000))

	# Create CarSetCondition
	#cond2 = storyboard.CarSetCondition({"flow1.0", "flow0.1", "flow0.2"})
	cond2 = storyboard.CarSetCondition({"host_veh"})

	# Create SpeedEffect
	effect0 = storyboard.SpeedEffect(200.44)

	# Create AndConditions
	and0 = storyboard.AndCondition(cond0, cond1)
	and1 = storyboard.AndCondition(and0, cond2)

	# Create OrCondition
	cond3 = storyboard.TimeCondition(timeline.seconds(190))
	or0 = storyboard.OrCondition(cond3, and1)

	# Create Story
	story = storyboard.Story(or0, [effect0])

	# Create Story 2
	cond4 = storyboard.TimeCondition(timeline.seconds(50), timeline.seconds(60))
	effect1 = storyboard.SpeedEffect(10.44)
	story1 = storyboard.Story(cond4, [effect1])

	# Create Story 3, overlapping story0
	cond5 = storyboard.TimeCondition(timeline.seconds(200), timeline.seconds(210))
	cond6 = storyboard.CarSetCondition({"host_veh", "eebl_veh"})
	and2 = storyboard.AndCondition(cond5, cond6)
	effect2 = storyboard.SpeedEffect(0.1)
	story2 = storyboard.Story(and2, [effect2])

	# Create Story 4, SpeedConditionGreater
	cond7 = storyboard.SpeedConditionGreater(4.0)
	cond8 = storyboard.TimeCondition(timeline.seconds(20), timeline.seconds(30))
	and3 = storyboard.AndCondition(cond7, cond8)
	effect3 = storyboard.SpeedEffect(1.0)
	story3 = storyboard.Story(and3, [effect3])

	# Register Stories at the Storyboard
	board.registerStory(story)
	#board.registerStory(story1)
	#board.registerStory(story2)
	#board.registerStory(story3)

	print("Stories loaded!")
	'''

