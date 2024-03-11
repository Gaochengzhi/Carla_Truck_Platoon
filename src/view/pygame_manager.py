import pygame
import os
import time
import numpy as np
import collections
import carla
from agent.baseAgent import BaseAgent
from util import waypoints_center, get_ego_vehicle, connect_to_server, thread_process_vehicles,time_const, log_time_cost
from view.color import WHITE, RED, GREEN, BLUE, BLACK, YELLOW, PURPLE
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
_view_offset_x, _view_offset_y = 0, 0 



class PyGameAgent(BaseAgent):
    def __init__(self, urban_waypoints, config):
        super().__init__("PyGame", config, config["pygame_port"])
        self.config = config
        self.urban_waypoints = urban_waypoints
        self.init_pygame_parameters(config)

    def init_pygame_parameters(self, config):
        self.width = config["screen_width"]
        self.height = config["screen_height"]

        

   

    def init_game_view(self, world, urban_waypoints, config):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.font = pygame.font.Font(None, 20)
        self.clock = pygame.time.Clock()
        self.centerMap = CenterMap(world, world.get_map(), self.screen,config)

    def init_game_camera(self, world, urban_waypoints, vehicle):
        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute('image_size_x', str(self.screen_main_width))
        camera_bp.set_attribute('image_size_y', str(self.screen_main_height))
        camera_bp.set_attribute('fov', '90')
        camera_location = carla.Location(x=0, y=0, z=0)
        self.camera = world.spawn_actor(
            camera_bp,
            carla.Transform(camera_location + carla.Location(z=100), carla.Rotation(pitch=-90)),
            attach_to=vehicle,
        )
        self.camera.listen(self.update_pygame_frame)

    def image2pygame_surface(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
        array = array[:, :, :3]  # Ignore alpha for RGB
        array = array[:, :, ::-1]  # Convert from BGR to RGB
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        return surface

    def update_pygame_frame(self, image):
        self.pygame_frame = self.image2pygame_surface(image)

    def draw_data_curve(self, data_history, color, rect, title):
        # Set background to light gray
        pygame.draw.rect(self.screen, (211, 211, 211), rect)

        # Draw the title for the box
        title_surface = self.font.render(title, True, BLACK)
        self.screen.blit(title_surface, (rect.x+5, rect.y - 30))

        if len(data_history) > 1:
            max_value = max(data_history)
            min_value = min(data_history)
            range_value = max_value - min_value if max_value - min_value else 1

            # Draw each point as a small rectangle
            point_width = 3
            for i, data in enumerate(data_history):
                x = rect.x + i * (rect.width / (len(data_history) - 1)) - point_width / 2
                y = rect.y + rect.height - (data - min_value) / range_value * rect.height
                pygame.draw.rect(self.screen, color, (x, y, point_width, rect.height - y + rect.y))

        # Draw axes
        self.draw_axes(rect)
    def draw_data_curveBACK(self, data_history, color, rect):
        if len(data_history) > 1:
            max_value = max(data_history)
            min_value = min(data_history)
            range_value = max_value - min_value if max_value - min_value else 1
            scaled_points = [(rect.x + i * rect.width / (len(data_history) - 1),
                              rect.y + rect.height - (data - min_value) / range_value * rect.height)
                             for i, data in enumerate(data_history)]
            pygame.draw.aalines(self.screen, color, False, scaled_points)
    def draw_axes(self, rect):
        pass

    @time_const(fps=30)
    # @log_time_cost
    def run_step(self, world):

        self.centerMap.tick()
        pygame.display.flip()
        # titles = ["","Acceleration", "Vertical Acceleration", "Yaw Rate", "Lateral Roll Angle"]


        # # Draw curves for each data type
        # data_histories = [self.acceleration_history, self.vertical_acceleration_history, 
        #                   self.yaw_rate_history, self.lateral_roll_angle_history]
        # for i, history in enumerate(data_histories):
        #     self.draw_data_curve(history, self.graph_colors[i], self.left_rects[i], titles[i])

        # ego_vehicle = get_ego_vehicle(world)
        # self.get_vehicle_data(ego_vehicle)
        # batch_process_vehicles(world, ego_vehicle, 200, [-100, 100], self.change_lane)

        # if self.pygame_frame:
        #     self.screen.blit(self.pygame_frame, self.pygame_frame_rect.topleft)
        #     dt = self.clock.tick()
        #     frame_rate = self.clock.get_fps()
        #     fps_text = self.font.render(f'Frame Rate: {frame_rate:.2f} FPS', True, BLACK)
        #     self.screen.blit(fps_text, (10, 10))
        #     

    def get_vehicle_data(self, ego_vehicle):
        acceleration = ego_vehicle.get_acceleration()
        angular_velocity = ego_vehicle.get_angular_velocity()
        transform = ego_vehicle.get_transform()

        self.acceleration_history.append(acceleration.length())
        self.vertical_acceleration_history.append(acceleration.x)
        self.yaw_rate_history.append(angular_velocity.z)
        self.lateral_roll_angle_history.append(transform.rotation.roll)


    def run(self):
        client, world = connect_to_server(self.config)
        self.start_agent()
        self.init_game_view(world, self.urban_waypoints, self.config)
        while True:
            self.run_step(world)

    def close(self):
        pygame.quit()
        if self.camera:
            self.camera.destroy()

# The rest of your original code...
