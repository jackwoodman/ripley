# ripley


import time
import math
import copy
import random
from pynput import keyboard

global message_list
global features


# constants
BOARD_SIDE_LENGTH = 51
M_S_L = 1001
DENSITY_MULT = 0.12
X_DISPLACE, Y_DISPLACE = BOARD_SIDE_LENGTH // 2, BOARD_SIDE_LENGTH // 2
pos_x, pos_y = M_S_L // 2, M_S_L // 2

feature = {
    'empty' : ' ',
    'mountain' : '^',
    'rock' : '#'
}

particle = {
    "explosion" : "*"
}

entity = {
    'player' : '@',
    'hunter' : '&',
    'villager': 'β'
}

entities = [
    ('player',[pos_x, pos_y])
]

features = {
    'empty'   : {'count':0, 'instances':{}},
    'mountain': {'count':0, 'instances':{}},
    'rock'    : {'count':0, 'instances':{}}
}

def estimate_loading(map_size):
    # model
    approx_time = (((0.2191*map_size**2) - (0.2*map_size) + (0.8264)) / 1000000) * 2

    if approx_time > 60:
        print("approximately " + str("{:.1f}".format(approx_time/60)) + " minutes")
    else:
        print("approximately " + str(int(approx_time)) + " seconds")

def node_map(LENGTH):
    NODES = []
    for row in range(M_S_L):
        new_obs = []
        for col in range(M_S_L):
            new_obs.append([(row, col),0,[0,0]])
        NODES.append(new_obs)
    return NODES

def node_set(LENGTH):
    NODES = []
    for row in range(M_S_L):
        for col in range(M_S_L):
            NODES.append((row, col))
    return NODES

def randomise_map(MAP, OBS, element_count):
    global features
    for new in range(int(element_count*0.32)):
        rand_x, rand_y = random.randint(2, M_S_L-1), random.randint(0, M_S_L-1)
        x,y = rand_x, rand_y
        possibles = [
            (x, y),   (x + 1, y), (x - 1, y),
            (x, y+1),  (x, y-1), (x+1, y+1),
            (x-1,y-1), (x+1,y-1), (x-1,y-1)]

        # this is really bad code pls ignore it
        weights = [0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,7,8]

        for i in range(random.randint(0, random.choice(weights))):
            n_x, n_y = possibles[i]
            if (n_x >= M_S_L) or (n_y >= M_S_L):
                continue
            features['mountain']['instances'][features['mountain']['count']] = (n_x, n_y)
            features['mountain']['count'] += 1
            MAP[n_x][n_y] = feature['mountain']
            OBS[n_x][n_y] = 1

    for new in range(element_count // 35):
        rand_x, rand_y = random.randint(0, M_S_L-1), random.randint(0, M_S_L-1)
        features['rock']['instances'][features['rock']['count']] = (rand_x, rand_y)
        features['rock']['count'] += 1
        MAP[rand_x][rand_y] = feature['rock']
        OBS[rand_x][rand_y] = 1
    return (MAP, OBS)

# first element is p_id, all others are particles of [id, coord, type, age, lifespan,]
def add_overlay(MAP, OBS, entity_list, particle_list):
    copy_map = copy.copy(MAP)
    copy_obs = copy.copy(OBS)

    # load particles into map overlay
    for part in particle_list[1:]:
        p_x, p_y = part[1]
        p_type = part[2]
        copy_map[p_x][p_y] = particle[p_type]

    # load entities into map overlay
    for ent in entity_list[::-1]:
        ent_co, ent_type = ent[2], ent[1]

        # check entity isn't dead
        if ent_type != "EMPTY":
            ent_x, ent_y = ent_co
            copy_map[ent_x][ent_y] = entity[ent_type]
            copy_obs[ent_x][ent_y] = 1

    return (copy_map, copy_obs)

def print_map(map, ent_list, mask=False, mask_size=0):
    # function to display compiled map
    global message_list
    centre_x, centre_y = ent_list[0][2]
    initial_corner = [centre_x - X_DISPLACE, centre_y - Y_DISPLACE]

    board_row, board_col = initial_corner

    board = []
    print(" "+BOARD_SIDE_LENGTH * "_")
    messages = 0
    message_limit = len(message_list)
    message_list.reverse()

    for x in range(board_row, board_row + BOARD_SIDE_LENGTH):
        current_row = "|"
        for y in range(board_col, board_col + BOARD_SIDE_LENGTH):
            try:
                if not mask:
                    current_row += MAP[x][y]
                else:
                    if coord_dist((centre_x, centre_y), (x,y)) <= mask_size:
                        current_row += MAP[x][y]
                    else:
                        current_row += " "
            except:
                g=input("ERROR: map out of bounds. Haven't fixed this yet :(")
                g=input("you're going to have to quit man")
        current_row += "|"
        board.append(current_row)

    for row in board:
        print(row, end = "")
        if messages < message_limit:
            print("  ",end="")
            new_message = message_list.pop()
            print(new_message,end="")

            messages += 1
        print("")
    print(" "+BOARD_SIDE_LENGTH * "-")

def spawn_entity(type, location, entity_tracker):
    # add new entity to entity tracker

    entity_id = len(entity_tracker)

    # check if any dead entities exist, replace with new entity if they do
    for element in entity_tracker:
        if element[1] == "EMPTY":
            entity_id = element[0]
            break

    if entity_id == len(entity_tracker):
        entity_tracker.append([])
    entity_tracker[entity_id] = [entity_id, type, location, ""]

    return entity_tracker

def get_all(entity_tracker, name):
    # get list of all entities of given type
    new_list = []
    for pot in entity_tracker:
        if pot[1] == name:
            new_list.append(pot)
    return new_list

def coord_dist(start, finish):
    # euclidean distance formula between two points
    s_x, s_y = start
    t_x, t_y = finish
    x = (t_x) - (s_x)
    y = (t_y) - (s_y)
    dist = math.sqrt(x**2+y**2)

    return dist

def check_OBS_collision(x, y, OBS):
    # return the value of the OBS cell for (x, y)
    return OBS[x][y]

def best_move(start_coord, target_coord, neighbours, current_recursion, MAX_RECURSION, OBS):
    global player_hunter
    global message_list


    # build list of neighbours and associated distances
    if (current_recursion == MAX_RECURSION) or (len(neighbours) == 0):
        #g=input("Returning " + str(start_coord))
        if len(message_list) < 3:
            message_list.append("MX RCRS: " + str(current_recursion))
        return start_coord

    # create list of distances from neighbours to target
    f_dists, n_dists = [], []
    for coord in neighbours:
        n_dist = (coord, coord_dist(coord, target_coord))
        n_dists.append(n_dist)

    n_dists = sorted(n_dists, key = lambda x: x[1])

    # get best distance
    baseline = n_dists[0][1]

    # find all with same best score as baseline
    for dist in n_dists:
        distance = dist[1]
        # check each drawn distance
        if (distance <= baseline + 2) :
            r_neighbours = get_neighbours(dist[0], OBS)
            # prevent selecting a dead end
            if len(r_neighbours) == 0:
                f_dists.append((dist[0],distance*10,start_coord))
                continue

            # check if found target
            if dist[0] == target_coord:
                f_dists.append((r_move, coord_dist(r_move, target_coord), distance))
                break

            else:
                # apply recursion to each drawn neighbour
                r_move = best_move(dist[0], target_coord, r_neighbours, current_recursion + 1, MAX_RECURSION, OBS)

                # save new node along with parent node
                f_dists.append((r_move, coord_dist(r_move, target_coord), dist[0]))

        else:
            # not drawn, reinsert node into new list
            f_dists.append((dist[0],distance,dist[0]))

    if len(f_dists) == 0:
        return start_coord

    # re-sort by distance to deepest child, and then return the parent of closest deep child
    f_dists = sorted(f_dists, key = lambda x: x[1])
    return f_dists[0][2]

def next_step(start_coord, target_coord, OBS, override_recursion=0):
    global message_list
    global player_hunter
    s_x, s_y = start_coord
    t_x, t_y = target_coord
    player_hunter = (start_coord, target_coord, OBS)
    # calculate required recursion limit based on the density of the map
    if override_recursion == 0:
        if DENSITY_MULT < 0.1:
            recursion_limit = 5
        elif DENSITY_MULT < 0.2:
            recursion_limit = 6
        elif DENSITY_MULT < 0.3:
            recursion_limit = 7
        else:
            recursion_limit = 8
    else:
        recursion_limit = override_recursion

    # find initial distance
    c_dist = coord_dist(start_coord, target_coord)

    # find all neighbours of current location
    coord_neighbours = get_neighbours(start_coord, OBS)

    # if no unobstructed neighbours, don't move
    if len(coord_neighbours) == 0:
        return (0, 0)


    message_list.append("PLYR: " + str(target_coord))
    message_list.append("HNTR: " + str(start_coord))

    # begin recursive search for best move
    recursion_depth = 0

    new_dir = best_move(start_coord, target_coord, coord_neighbours, recursion_depth, recursion_limit, OBS)

    # apply best move to a delta for move function
    move_delta = (-1*(s_x - new_dir[0]), -1*(s_y - new_dir[1]))
    message_list.append("R_LIM: " + str(recursion_limit))
    message_list.append("DELT: " + str(move_delta))
    message_list.append("TRGT: " + str(new_dir))
    message_list.append("C_DIST: " + "{:.2f}".format(c_dist))
    message_list.append("N_DIST: " + "{:.2f}".format(coord_dist(new_dir, target_coord)))
    message_list.append("  ")
    message_list.append("PARTICLES: " + str(len(particle_tracker)-1))

    return move_delta

def wander(co_ord, OBS):
    # return a random wander delta
    move_chance = random.randint(0,9)
    move = (True if move_chance > 3 else False)

    if not move:
        return (0, 0)

    return calculate_delta(random_direction())

def villager_AI(entity_tracker, OBS, e_history):
    global message_list
    MOVE_CHANCE = 5
    move_list = []
    for i in range(MOVE_CHANCE):
        move_list.append(i)

    for villager in get_all(entity_tracker, 'villager'):
        entity_id, villager_x, villager_y = villager[0], villager[2][0], villager[2][1]
        player = entity_tracker[0][2]
        c_distance = coord_dist(villager[2], player)

        e_history = history_update(e_history, entity_id, villager[2])

        if random.randint(0, 10) in move_list:
            next_move = wander(villager[2], OBS)
            entity_tracker, OBS = move_entity(entity_tracker,OBS, entity_id, next_move)

    return entity_tracker, OBS, e_history








def hunter_AI(entity_tracker, OBS, e_history):
    # function to handle Ai states for each hunter in tracker
    global message_list


    for hunter in get_all(entity_tracker, 'hunter'):
        entity_id = hunter[0]
        hunter_x, hunter_y = hunter[2]
        player = entity_tracker[0][2]
        current_distance = coord_dist(hunter[2], player)

        # decide if to hunter player or to wander
        if len(entity_tracker[entity_id]) < 4:
            entity_tracker[entity_id].append("EMPTY")
        if current_distance < 20:
            entity_tracker[entity_id][3] = "hunting"
        else:
            entity_tracker[entity_id][3] = "wandering"

        message_list.append("AI " + str(entity_id)+": " + entity_tracker[entity_id][3])
        message_list.append("DIST: " + str(int(current_distance)))

        # decided whether stuck or not
        e_history = history_update(e_history, entity_id, hunter[2])
        if check_stuck(e_history, entity_id):
            next_move = next_step(hunter[2], player, OBS, override_recursion=8)
        else:
            state = entity_tracker[entity_id][3]
            if state == "hunting":
                next_move = next_step(hunter[2], player, OBS)
            elif state == "wandering":
                next_move = wander(hunter[2], OBS)
        entity_tracker, OBS = move_entity(entity_tracker,OBS, entity_id, next_move)

    return entity_tracker, OBS, e_history


def check_collision(co_ord):
    for key, value in feature.items():
        if value == MAP[co_ord[0]][co_ord[1]]:
            if tuple(co_ord) in features[key]['instances'].values():
                return True
    return False

def check_bounds(co_ord):
    # function to check if co_ord is within the game map or not
    x, y = co_ord
    if (0 <= x <= M_S_L-1):
         if (0 <= y <= M_S_L-1):
             # whole co_ord is within map
             return True
         else:
             # y co_ord is outside map
             return -1
    # x co_ord is outside map
    return -2

def get_neighbours(node, OBS):
    # get list of neighbours of 'node', checking for collision
    n_x, n_y = node
    neighbours = []

    p_ns = [
        (n_x + 1, n_y), (n_x - 1, n_y),
        (n_x, n_y + 1), (n_x, n_y - 1)
    ]

    for pn in p_ns:
        pn_x, pn_y = pn
        if (pn_x >= 0 and pn_y >= 0):
            if (pn_x < M_S_L and pn_y < M_S_L):
                if OBS[pn_x][pn_y] == 0:
                    neighbours.append(pn)
    return neighbours

def kill_entity(entity_tracker, e_history, entity_id):
    death_state = [entity_id, "empty", (0, 0), "dead"]


    entity_tracker[entity_id] = death_state
    e_history[entity_id] = []
    return entity_tracker, e_history


def move_entity(entity_tracker, OBS, ent_id, delta):

    entity_x, entity_y = entity_tracker[ent_id][2]


    x_displace, y_displace = delta

    potential_coord = [entity_x + x_displace, entity_y + y_displace]

    # check coord is within gamemap
    in_bounds = check_bounds(potential_coord)

    if in_bounds == -1:
        potential_coord[1] -= 2*y_displace
    elif in_bounds == -2:
        potential_coord[0] -= 2*x_displace

    if not check_collision(potential_coord):
        OBS[entity_x][entity_y] = 0
        OBS[potential_coord[0]][potential_coord[1]] = 1
        entity_tracker[ent_id][2] = potential_coord
    return (entity_tracker, OBS)


def random_direction():
    # pick random cardinal direction
    dirs = ['w','a','s','d']
    return dirs[random.randint(0,3)]

def calculate_delta(direction):
    # take cardinal direction, output the delta
    directions_equiv = {
        "w" : (-1, 0),
        "s": (1, 0),
        "a"   : (0, -1),
        "d" : (0, 1)
    }
    return directions_equiv[direction]

def compile_entities(entities):
    # compile all entities in starting list into entity tracker
    new_list = []
    ent_count = 0

    default_state = {
        "hunter" : "wandering",
        "human"  : "wandering",
        "player" : "playing",
        "villager": "wandering"
    }

    for en in entities:
        # assign ID to each entity before insertion into tracker
        ent_name, ent_co = en
        ent = [ent_count, ent_name, ent_co, default_state[ent_name]]
        new_list.append(ent)
        ent_count += 1

    return new_list

def last_overlay(entity_tracker, particle_tracker):
    # build list of entities and their current locations
    new_l = []

    for ent in entity_tracker:
        new_l.append([ent[0],ent[2]])

    for part in particle_tracker[1:]:
        new_l.append([part[0], part[1]])

    return new_l

def strip_overlay(MAP, last_list):
    # remove entities & particles from the previous frame from the map
    for element in last_list:
        # set element location to read empty - these can also be particles but whatever
        el_x, el_y = element[1]
        MAP[el_x][el_y] = feature['empty']

    return MAP

def random_location(entities, dist_from_player=10):
    # chooses a random location a given distance from the player
    choice = [1, -1]
    multi1, multi2 = random.choice(choice), random.choice(choice)
    player_x, player_y = entities[0][2]

    # specific distance selected
    if dist_from_player > 0:
        random_coord = [player_x + (multi1 * dist_from_player), player_y + (multi2 * dist_from_player)]

    # can be placed anywhere on the map
    else:
        M_BOUND = M_S_L-1
        random_coord = [random.randint(0, M_BOUND), random.randint(0, M_BOUND)]

    return random_coord

def keyboard_checker(event, char):
    if event.key == keyboard.KeyCode.from_char(char):
        return True

def cooldown(last_called, duration):
    start_time = time.time()
    while True:
        if abs(start_time - last_called) > duration:
            break

def get_adjacent(coord):
    # get all adjacent coords, including diagonal
    x, y = coord
    adj_list = [
        (x-1, y+1), (x, y+1), (x+1, y+1),
        (x-1, y  ),           (x+1, y  ),
        (x-1, y-1), (x, y-1), (x+1, y-1)
    ]
    return adj_list

def keyboard_input():
    # take watches for valid input on keyboard
    with keyboard.Events() as events:
        # Block for as much as possible
        choices = ['w','a','s','d']
        event = events.get(1e6)
        for possible in choices:
            if keyboard_checker(event, possible):
                return possible

        return "!"

def input_logic():
    # calculate delta based on keyboard input
    new_com = keyboard_input()
    if new_com == "!":
        new_delta = (0, 0)
    else:
        element = new_com[0]
        new_delta = calculate_delta(element)   # calculate next coord

    return new_delta

def history_update(h_dict, ent_id, coord, limit=7):
    global message_list
    history = h_dict[ent_id]
    history_len = len(history)

    # update history with new coord
    if history_len < limit:
        history.insert(0, coord)
        h_dict[ent_id] = history
    else:
        # move history events forward
        n_h = []
        for event in range(0, limit - 1):
            n_h.append(history[event+1])

        # set last
        n_h.append(coord)
        # update h_dict with new history
        h_dict[ent_id] = n_h

    message_list.append("HIST: " + str(history_len))
    return h_dict


def check_stuck(h_dict, ent_id):
    global message_list
    history = h_dict[ent_id]
    occurance_dict = {}

    # build occurance_dict
    for e_x, e_y in history:
        # create new instance in dict
        if e_x not in occurance_dict:
            occurance_dict[e_x] = {}
            occurance_dict[e_x][e_y] = 1
        else:
            # check if y coord also matches
            if e_y not in occurance_dict[e_x]:
                occurance_dict[e_x][e_y] = 1
            else:
                # increment dict, return True if this makes majority
                occurance_dict[e_x][e_y] += 1

                # passed lost test
                if occurance_dict[e_x][e_y] >= 3:
                    message_list.append("LOST: True")
                    return True

    message_list.append("LOST: False")
    return False

def command_watchdog(previous_command):
    while abs(time.time() - previous_command) < 0.13:
        continue


def add_particle(symbol, co_ord, duration, particle_tracker):
    new_particle = [particle_tracker[0], co_ord, symbol, 0, duration]

    particle_tracker[0] += 1
    particle_tracker.append(new_particle)
    return particle_tracker

def age_particles(particle_tracker):
    new_tracker = [particle_tracker[0]]
    for part in particle_tracker[1:]:
        age, lifespan = part[3], part[4]


        if age < lifespan:
            part[3] += 1
            new_tracker.append(part)


    return new_tracker

#===============================================================================

# start of main
print("\n -- RIPLEY --")
print(" map size: "+str(M_S_L)+" x "+str(M_S_L))
print(" initial entities: " + str(len(entities)))
print(" density multiplier: " + str(DENSITY_MULT)+str("x"))
print(" now loading: ",end="")
estimate_loading(M_S_L)


# generate empty MAP and OBS
MAP, OBS, e_history, message_list, object = [], [], {}, [], {}
for row in range(M_S_L):
    new_row, new_obs = [], []
    for col in range(M_S_L):
        new_row.append(feature['empty'])
        new_obs.append(0)
    MAP.append(new_row)
    OBS.append(new_obs)


# calculate the number of objects that should appear in map
goal_objects = int((M_S_L**2) * DENSITY_MULT)

# initialise map, obstacle tracker, and entity tracker
MAP, OBS = randomise_map(MAP, OBS, goal_objects)
entity_tracker = compile_entities(entities)
particle_tracker = [1] # first element is p_id, all others are particles of [id, coord, type, age, lifespan,]

# intro animation
for view in range(0, BOARD_SIDE_LENGTH):
    time.sleep(0.05)
    print_map(add_overlay(MAP,OBS,entity_tracker, particle_tracker)[0], entity_tracker, True, view)


# main game loop, start timer
last_move = time.time()

#for i in range(100000):
#    particle_tracker = add_particle("explosion", random_location(entity_tracker, 0), 5+random.randint(0 ,60), particle_tracker)


# intial demo stuff
test_location = random_location(entity_tracker, dist_from_player=15)
entity_tracker = spawn_entity("hunter", test_location, entity_tracker)
for i in range(5):
    entity_tracker = spawn_entity("villager", random_location(entity_tracker, random.randint(10,50)),entity_tracker)


# create history record for lost detection & pathfinding
for ent in entity_tracker:
    e_history[ent[0]] = []

while True:
    # display map and hold for move delay
    print_map(add_overlay(MAP,OBS,entity_tracker, particle_tracker)[0], entity_tracker)
    # prevent command overlap
    command_watchdog(last_move)

    # keep track of last positions of entities
    last_list = last_overlay(entity_tracker, particle_tracker)

    # get new player move
    player_move = input_logic()

    # update map with new player move
    entity_tracker, OBS = move_entity(entity_tracker, OBS, 0, player_move)

    # age particles in particle tracker
    particle_tracker = age_particles(particle_tracker)

    # remove previous entities from map for computation and delay
    MAP = strip_overlay(MAP, last_list)

    # run hunter AI driver code
    entity_tracker, OBS, e_history = hunter_AI(entity_tracker, OBS, e_history)

    # run villager AI driver code
    entity_tracker, OBS, e_history = villager_AI(entity_tracker, OBS, e_history)


    # reset timer
    last_move = time.time()
