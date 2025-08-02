import pygame

class GUI:
    def __init__(self, controller):
        self.controller = controller
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        self.font = pygame.font.Font(None, 36)
        
        # Arm/Disarm button
        self.button_rect = pygame.Rect(100, 100, 150, 50)
        self.button_text = "Arm"
        self.button_color = (0, 255, 0)

        # Enable FIP controller button (default is navigation control)
        self.button_FIP = pygame.Rect(300, 100, 150, 50)
        self.button_FIP_text = "FIP"
        self.button_FIP_color = (0, 255, 0)

        # Land button
        self.button_land = pygame.Rect(500, 100, 150, 50)
        self.button_land_text = "Land"
        self.button_land_color = (0, 255, 0)
        


    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.controller.on_close()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Arm/Disarm button
                if self.button_rect.collidepoint(event.pos):
                    self.controller.get_logger().info(f'Button press happened state {self.controller.armed}')
                    if not self.controller.armed:
                        self.controller.armed = True
                        self.update_button("Disarm", (255, 0, 0))
                    else: 
                        self.controller.armed = False
                        self.update_button("Arm", (0, 255, 0))
                
                # FIP button 
                if self.button_FIP.collidepoint(event.pos):
                    if not self.controller.useFIP:
                        self.controller.useFIP = True
                        self.update_FIP_button("Nav", (255, 0, 0))
                    else: 
                        self.controller.useFIP = False
                        self.update_FIP_button("FIP", (0, 0, 255))

                # land button 
                if self.button_land.collidepoint(event.pos):
                    if not self.controller.land:
                        self.controller.land = True
                   
                

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE: 
                    self.controller.armed = False 
                    self.update_button("Arm", (0, 255, 0))

        self.screen.fill((0, 0, 0))
        
        # Draw Arm/Disarm button
        pygame.draw.rect(self.screen, self.button_color, self.button_rect)
        text_surface = self.font.render(self.button_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=self.button_rect.center)
        self.screen.blit(text_surface, text_rect)

        # Draw FIP button
        pygame.draw.rect(self.screen, self.button_FIP_color, self.button_FIP)
        FIP_text_surface = self.font.render(self.button_FIP_text, True, (255, 255, 255))
        FIP_text_rect = FIP_text_surface.get_rect(center=self.button_FIP.center)
        self.screen.blit(FIP_text_surface, FIP_text_rect)

        # Draw land button
        pygame.draw.rect(self.screen, self.button_land_color, self.button_land)
        land_text_surface = self.font.render(self.button_land_text, True, (255, 255, 255))
        land_text_rect = land_text_surface.get_rect(center=self.button_land.center)
        self.screen.blit(land_text_surface, land_text_rect)


        pygame.display.flip()

    def update_button(self, text, color):
        self.button_text = text
        self.button_color = color

    def update_FIP_button(self, text, color):
        self.button_FIP_text = text
        self.button_FIP_color = color

    '''def update_land_button(self, text, color):
        self.button_land_text = text
        self.button_land_color = color'''

    def quit(self):
        pygame.quit()
