import logging
import sys
from util import Singleton
import yaml
import time
import os
from colorlog import ColoredFormatter


def init_logger(state="console"):
    current_time = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())

    log_file = f"../log/{current_time}/log.txt"
    formatter = ColoredFormatter(
        "%(log_color)s%(levelname)-8s%(reset)s %(asctime)s %(message)s",
        datefmt="%H:%M:%S",
        reset=True,
        log_colors={
            'DEBUG': 'cyan',
            'INFO': 'green',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'red',
        }
    )

    logger = logging.getLogger()

    if state in ["file", "all"]:
        os.makedirs(f"../log/{current_time}", exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    if state in ["console", "all"]:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    if state != "none":
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.CRITICAL)


class Config(metaclass=Singleton):
    def __init__(self):
        with open('../config/base.yml', 'r') as base_config_file:
            self.config = yaml.safe_load(base_config_file)

    def update_config(self, updates, current_dict=None):
        """
        Recursively update the configuration dictionary with the provided updates.

        :param updates: A dictionary containing the updates.
        :param current_dict: The current level in the configuration dictionary.
        """
        if current_dict is None:
            current_dict = self.config

        for key, value in updates.items():
            if isinstance(value, dict) and key in current_dict:
                self.update_config(value, current_dict[key])
            else:
                current_dict[key] = value

    def update_dict_recursively(self, d, u):
        for k, v in u.items():
            if isinstance(v, dict):
                d[k] = self.update_dict_recursively(d.get(k, {}), v)
            else:
                d[k] = v
        return d

    def merge(self, args):
        update_params = {}
        if args.debug:
            with open("../config/"+args.debug+".yml", 'r') as test_config_file:
                update_params = yaml.safe_load(test_config_file)
        if args.town:
            update_params["map_name"] = args.town
        if args.log:
            init_logger(args.log)
        else:
            init_logger("none")

        logging.info("Config init")

        self.config = self.update_dict_recursively(self.config, update_params)
        return self.config


config = Config()
