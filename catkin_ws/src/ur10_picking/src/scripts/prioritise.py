#!/usr/bin/env python

import json

all_items = ["feline_greenies_dental_treats",
             "expo_dry_erase_board_eraser",
             "genuine_joe_plastic_stir_sticks",
             "laugh_out_loud_joke_book",
             "kyjen_squeakin_eggs_plush_puppies",
             "dr_browns_bottle_brush",
             "paper_mate_12_count_mirado_black_warrior",
             "sharpie_accent_tank_style_highlighters",
             "highland_6539_self_stick_notes",
             "cheezit_big_original",
             "mommys_helper_outlet_plugsl",
             "rolodex_jumbo_pencil_cup",
             "safety_works_safety_glasses",
             "kong_air_dog_squeakair_tennis_ball",
             "kong_duck_dog_toy",
             "champion_copper_plus_spark_plug",
             "oreo_mega_stuf",
             "mead_index_cards",
             "elmers_washable_no_run_school_glue",
             "stanley_66_052",
             "munchkin_white_hot_duck_bath_toy",
             "kong_sitting_frog_dog_toy",
             "first_years_take_and_toss_straw_cup",
             "mark_twain_huckleberry_finn",
             "crayola_64_ct"]

item_profiles = {"feline_greenies_dental_treats": {"hookability": 1, "suckability": 2/6},
                 "expo_dry_erase_board_eraser": {"hookability": 1, "suckability": 1},
                 "genuine_joe_plastic_stir_sticks": {"hookability": 1, "suckability": 1},
                 "laugh_out_loud_joke_book": {"hookability": 0, "suckability": 1/6}, # Not in original list object list from gripper team
                 "kyjen_squeakin_eggs_plush_puppies": {"hookability": 1, "suckability": 1/6},
                 "dr_browns_bottle_brush": {"hookability": 1, "suckability": 1/6},
                 "paper_mate_12_count_mirado_black_warrior": {"hookability": 0, "suckability": 2/6},
                 "sharpie_accent_tank_style_highlighters": {"hookability": 1, "suckability": 1/6},
                 "highland_6539_self_stick_notes": {"hookability": 1, "suckability": 1},
                 "cheezit_big_original": {"hookability": 0, "suckability": 1},
                 "mommys_helper_outlet_plugs": {"hookability": 1, "suckability": 5/6},
                 "rolodex_jumbo_pencil_cup": {"hookability": 1, "suckability": 1/6},
                 "safety_works_safety_glasses": {"hookability": 0, "suckability": 0}, # Not going to be included
                 "kong_air_dog_squeakair_tennis_ball": {"hookability": 1, "suckability": 1/6},
                 "kong_duck_dog_toy": {"hookability": 0, "suckability": 0},
                 "champion_copper_plus_spark_plug": {"hookability": 1, "suckability": 2/6},
                 "oreo_mega_stuf": {"hookability": 0, "suckability": 5/6},
                 "mead_index_cards": {"hookability": 0, "suckability": 0},
                 "elmers_washable_no_run_school_glue": {"hookability": 1, "suckability": 2/6}, # Not in original list object list from gripper team
                 "stanley_66_052": {"hookability": 0, "suckability": 1/6},
                 "munchkin_white_hot_duck_bath_toy": {"hookability": 1, "suckability": 1},
                 "kong_sitting_frog_dog_toy": {"hookability": 0, "suckability": 0}, # Not going to be included
                 "first_years_take_and_toss_straw_cup": {"hookability": 1, "suckability": 1},
                 "mark_twain_huckleberry_finn": {"hookability": 0, "suckability": 1/6},
                 "crayola_64_ct": {"hookability": 0, "suckability": 1}
                 }
# z diff is ~32cm
# y diff is ~27cm
bin_profiles = {"bin_A": {"location_score": 1/4, "shelf_offset": {"x": 0.0, "y": -27.0, "z": 32.0}},
                "bin_B": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": 0.00, "z": 32.0}},
                "bin_C": {"location_score": 1/4, "shelf_offset": {"x": 0.0, "y": 27.0, "z": 32.0}},
                "bin_D": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": -27.0, "z": 0.00}},
                "bin_E": {"location_score": 3/4, "shelf_offset": {"x": 0.0, "y": 0.00, "z": 0.00}},
                "bin_F": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": 27.0, "z": 0.00}},
                "bin_G": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": -27.0, "z": -32.0}},
                "bin_H": {"location_score": 3/4, "shelf_offset": {"x": 0.0, "y": 0.00, "z": -32.0}},
                "bin_I": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": 27.0, "z": -32.0}},
                "bin_J": {"location_score": 1/4, "shelf_offset": {"x": 0.0, "y": -27.0, "z": 64.0}},
                "bin_K": {"location_score": 2/4, "shelf_offset": {"x": 0.0, "y": 0.00, "z": 64.0}},
                "bin_L": {"location_score": 1/4, "shelf_offset": {"x": 0.0, "y": 27.0, "z": 64.0}}
                }


def import_json(filename):
    # Opening JSON file
    with open(filename, 'r') as openfile:
        # Reading from json file
        json_object = json.load(openfile)

    json_bin_contents = json_object["bin_contents"]
    json_work_order = json_object["work_order"]
    print(f' The imported bin_contents is {json_bin_contents}')
    print(f' The imported work_order is {json_work_order}')
    return json_object


def prioritise_items(items, order):
    # Score for number of items on shelf
    for bins in order:
        if len(items[bins["bin"]]) == 1:
            bins["score"] = 300
        elif len(items[bins["bin"]]) == 2:
            bins["score"] = 200
        else:
            bins["score"] = 100

    # print(work_order)

    for bins in order:
        # Hookability score: Do not need the if statement if the positive score remains
        # if item_profiles[bins["item"]]["hookability"] == 1:
            # bins["score"] += 10
        # Suckability score
        bins["score"] += item_profiles[bins["item"]]["suckability"]
        # Location Score
        bins["score"] += bin_profiles[bins["bin"]]["location_score"]

    # print(work_order)

    prioritised_order = sorted(work_order, key=lambda item: item["score"], reverse=True)

    print(order)

    return prioritised_order


if __name__ == "__main__":
    shelf_objects = import_json('apc_pick_test.json')

    bin_contents = shelf_objects["bin_contents"]
    work_order = shelf_objects["work_order"]

    prioritised_work_order = prioritise_items(bin_contents, work_order)
