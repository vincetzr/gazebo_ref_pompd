import argparse

available_problems = [
    'tiger',
    'rocksample',
    'mos',
    'tag',
    'load_unload',
    'gridworld'
]

def parse_args():
    parser = argparse.ArgumentParser(description="pomdp_py CLI")
    parser.add_argument("-r", "--run", type=str,
                        help="run a pomdp under pomdp_problems."
                        "Available options: {}".format(available_problems))
    args = parser.parse_args()
    return parser, args

if __name__ == "__main__":
    parser, args = parse_args()
    if args.run:
        if args.run.lower() == "tiger":
            from problems.tiger.tiger import main
            main()

        elif args.run.lower() == "rocksample":
            from problems.rocksample.rocksample import main
            main()

        # elif args.run.lower() == "mos":
        #     from problems.multi_object_search.problem import unittest
        #     unittest()

        # elif args.run.lower() == "tag":
        #     from problems.tag.problem import main
        #     main()

        elif args.run.lower() == "load_unload":
            from problems.load_unload.load_unload import main
            main()

        elif args.run.lower() == "gridworld":
            from problems.gridworld.gridworld import main
            main()

        else:
            print("Unrecognized pomdp: {}".format(args.run))

    else:
        parser.print_help()
