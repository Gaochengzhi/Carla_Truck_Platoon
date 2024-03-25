import multiprocessing
import threading
from data.commuicate_manager import CommuniAgent
import logging
from util import time_const


class BaseAgent(multiprocessing.Process):
    def __init__(self, agent_name, agent_port):
        super(BaseAgent, self).__init__()
        self.name = agent_name
        self.agent_port = agent_port

    def init_base_agent(self):
        self.communi_agent = self.init_communi_agent(
            self.name, self.agent_port)
        # self.start_main_listener_thread()

    def start_main_listener_thread(self):
        self.stop_listener = threading.Event()
        self.listener_thread = threading.Thread(
            target=self.listen_for_main_message)
        self.listener_thread.start()

    def listen_for_main_message(self):
        self.main_message_operation()

    @time_const(fps=1)
    def main_message_operation(self):
        while not self.stop_listener.is_set():
            main_msg = self.communi_agent.rec_obj("main")
            if main_msg == "end":
                logging.debug(f"{self.name} received {main_msg} message")
                self.close_agent()
                break

    def close_agent(self):
        self.communi_agent.close()
        self.stop_listener.set()
        self.listener_thread.join()
        self.listener_thread._stop()

    def init_communi_agent(self, agent_name, agent_port):
        communi_agent = CommuniAgent(agent_name)
        communi_agent.init_publisher(
            agent_port)
        communi_agent.send_obj(f"{agent_name} started")
        # communi_agent.init_subscriber("main",
        #                               self.config["main_port"])
        return communi_agent
