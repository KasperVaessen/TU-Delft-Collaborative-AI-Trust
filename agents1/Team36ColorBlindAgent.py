import copy
import json

from matrx.actions.door_actions import OpenDoorAction
from matrx.actions.object_actions import GrabObject, DropObject

from .Team36BaseAgent import BaseAgent, Phase
from typing import Dict

class ColorBlindAgent(BaseAgent):

    def __init__(self, settings:Dict[str,object]):
        super().__init__(settings)
        self._target_location = (11, 22) # location next to the middle target block

    def filter_bw4t_observations(self, state):
        # Remove color value from all observed items
        for key in state.keys():
            try:
                state[key]['visualization'].pop('colour')
            except KeyError:
                continue
        try:
            state.get_self()['is_carrying'][0]['visualization'].pop('colour')
        except IndexError:
            pass

        for teammate in self._teamMembers:
            if teammate != 'human':
                for item in state[teammate]['is_carrying']:
                    item['visualization'].pop('colour')

        return state

    def _handleMessages(self, state):
        # if a goal has been satisfied and you are carrying a block for that goal, drop the block
        missing_goals = self.get_missing_goals()
        is_goal = False
        for goal in missing_goals:
            if len(state.get_self()['is_carrying']) == 0:
                return
            carrying_vis = state.get_self()['is_carrying'][0]['visualization']
            if (carrying_vis['size'] == goal['visualization']['size']
                    and carrying_vis['shape'] == goal['visualization']['shape']
                    # and carrying_vis['colour'] == goal['visualization']['colour']
            ):
                is_goal = True

        if not is_goal:
            self._phase = Phase.DROP_BLOCK


    def decide_on_bw4t_action(self, state):
        state = self.filter_bw4t_observations(state)
        return super().decide_on_bw4t_action(state)


    def get_closest_possible_target(self, state):
        # Slaat de locaties op van het blok dat het laatst gevonden moet worden
        # en pakt de dichtsbijzijnde die gevonden is door het liefst een andere agent

        me = self._you['obj_id']
        targets = None
        # TODO: situatie toevoegen waar je de block hoogste trust prioritizeert
        if len(self._possible_targets) > 1:
            for k, v in self._possible_targets.items():
                if k != me:
                    targets = v
        else:
            for v in self._possible_targets.values():
                targets = v

        return sorted(targets, key=lambda b: self.dist(self._you, b, state))[0]

    def plan_verify_goals(self):
        # When planning to verify target, just plan a next action
        self._phase = Phase.PLAN_NEXT_ACTION

    def plan_next_action(self, state):
        self._navigator.reset_full()
        moving_to_target = False
        missing_goals = self.get_missing_goals()
        if len(missing_goals) > 1:
            missing_goals = missing_goals[1:][::-1]
        for i in range(len(missing_goals)):
            missing_goals[i] = missing_goals[i]['visualization']

        self._possible_targets = {}
        missing_index = len(missing_goals)
        for block in self._world_state['found_blocks']:
            if block['location'] != self._target_location and block['location'] not in [goal['location'] for goal in self._world_state['goals']]:
                try:
                    i = missing_goals.index(block['visualization'])

                    if i < missing_index:
                        missing_index = i
                        self._possible_targets = {block['by']: [block]}
                    elif i == missing_index:
                        self._possible_targets[block['by']].append(block)
                except KeyError:
                    self._possible_targets[block['by']] = [block]
                except ValueError:
                    pass

        if (len(state.get_self()['is_carrying']) < self._carrying_capacity
                and len(state.get_self()['is_carrying']) < len(missing_goals)
                and len(self._possible_targets) > 0):

            self._phase = Phase.PLAN_PATH_TO_BLOCK
            moving_to_target = True

        elif len(state[self.agent_id]['is_carrying']) > 0:
            self._phase = Phase.PLAN_PATH_TO_GOAL
            moving_to_target = True

        if not moving_to_target:
            self._phase = Phase.PLAN_PATH_TO_ROOM

    def pickup_block(self, agent_name, observations, block_id):
        self._phase = Phase.PLAN_NEXT_ACTION
        missing_goals = self.get_missing_goals()
        for block in observations['blocks']:
            for vals in self._possible_targets.values():
                for tar in vals:
                    if block['location'] == tar['location']:
                        for b in self._world_state['found_blocks']:
                            if b['visualization'] == block['visualization'] and b['location'] == block['location']:
                                self._world_state['found_blocks'].remove(b)
                                break

                        self._sendMessage(
                            'Picking up goal block {} at location {}'.format(json.dumps(block['visualization']),
                                                                             block['location']),
                            agent_name)
                        return GrabObject.__name__, {'object_id': block['obj_id']}
            if block['visualization'] == missing_goals[0]['visualization']:
                for b in self._world_state['found_blocks']:
                    if b['visualization'] == block['visualization'] and b['location'] == block['location']:
                        self._world_state['found_blocks'].remove(b)
                        break

                self._sendMessage(
                    'Picking up goal block {} at location {}'.format(json.dumps(block['visualization']),
                                                                     block['location']),
                    agent_name)
                return GrabObject.__name__, {'object_id': block['obj_id']}
        # bij observations, pak de missing die het laatst nodig is!

    def plan_path_to_goal(self, state):
        # Alway place the target block next to target location
        if len(state.get_self()['is_carrying']) == 0:
            self._phase = Phase.PLAN_NEXT_ACTION
        else:
            self._navigator.reset_full()
            self._navigator.add_waypoint(self._target_location)
            self._phase = Phase.FOLLOW_PATH_TO_GOAL

    def follow_path_to_goal(self, state, agent_name):
        self._state_tracker.update(state)
        action = self._navigator.get_move_action(self._state_tracker)
        if action is not None:
            return action, {}
        else:
            return self.drop_block(agent_name, state, state.get_self()['is_carrying'][0]['obj_id'])

    def drop_block(self, agent_name, state, block_id):
        self._phase = Phase.PLAN_NEXT_ACTION

        block_to_drop = list(filter(lambda b: b['obj_id'] == block_id, state.get_self()['is_carrying']))
        if len(block_to_drop) > 0:
            block_to_drop = block_to_drop[0]
        else:
            return None
        block_vis = {'size': block_to_drop['visualization']['size'], 'shape': block_to_drop['visualization']['shape']}
        self._sendMessage(
            'Dropped goal block {} at drop location {}'.format(json.dumps(block_vis), state.get_self()['location']),
            agent_name)
        return DropObject.__name__, {'object_id': block_to_drop['obj_id']}