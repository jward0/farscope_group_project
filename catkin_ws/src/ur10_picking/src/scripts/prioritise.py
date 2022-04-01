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

item_profiles = {"feline_greenies_dental_treats": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "expo_dry_erase_board_eraser": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "genuine_joe_plastic_stir_sticks": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "laugh_out_loud_joke_book": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "kyjen_squeakin_eggs_plush_puppies": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "dr_browns_bottle_brush": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "paper_mate_12_count_mirado_black_warrior": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "sharpie_accent_tank_style_highlighters": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "highland_6539_self_stick_notes": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "cheezit_big_original": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "mommys_helper_outlet_plugs": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "rolodex_jumbo_pencil_cup": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "safety_works_safety_glasses": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "kong_air_dog_squeakair_tennis_ball": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "kong_duck_dog_toy": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "champion_copper_plus_spark_plug": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "oreo_mega_stuf": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "mead_index_cards": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "elmers_washable_no_run_school_glue": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "stanley_66_052": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "munchkin_white_hot_duck_bath_toy": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "kong_sitting_frog_dog_toy": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "first_years_take_and_toss_straw_cup": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "mark_twain_huckleberry_finn": {"fragile": 1, "hookability": 0, "suckability": 0},
                 "crayola_64_ct": {"fragile": 1, "hookability": 0, "suckability": 0}
                 }

bin_profiles = {"bin_A": {"location": 1/4},
                "bin_B": {"location": 2/4},
                "bin_C": {"location": 1/4},
                "bin_D": {"location": 2/4},
                "bin_E": {"location": 3/4},
                "bin_F": {"location": 2/4},
                "bin_G": {"location": 2/4},
                "bin_H": {"location": 3/4},
                "bin_I": {"location": 2/4},
                "bin_J": {"location": 1/4},
                "bin_K": {"location": 2/4},
                "bin_L": {"location": 1/4}
                }


def import_json(filename):
    # Opening JSON file
    with open(filename, 'r') as openfile:
        # Reading from json file
        json_object = json.load(openfile)

    bin_contents = json_object["bin_contents"]
    work_order = json_object["work_order"]
    print(f' The imported bin_contents is {bin_contents}')
    print(f' The imported work_order is {work_order}')
    return json_object


def prioritise_items(bin_contents, work_order):
    # Score for number of items on shelf
    for bins in work_order:
        if len(bin_contents[bins["bin"]]) == 1:
            bins["score"] = 300
        elif len(bin_contents[bins["bin"]]) == 2:
            bins["score"] = 200
        else:
            bins["score"] = 100

    # print(work_order)

    for bins in work_order:
        # Hookability score: Do not need the if statement if the positive score remains
        if item_profiles[bins["item"]]["hookability"] == 1:
            bins["score"] += 10
        # Suckability score
        bins["score"] += item_profiles[bins["item"]]["suckability"]
        # Location Score
        bins["score"] += bin_profiles[bins["bin"]]["location"]

    # print(work_order)

    order = sorted(work_order, key=lambda item: item["score"], reverse=True)

    print(order)

    return order


if __name__ == "__main__":
    json_object = import_json('apc_pick_test.json')

    bin_contents = json_object["bin_contents"]
    work_order = json_object["work_order"]

    order = prioritise_items(bin_contents, work_order)
