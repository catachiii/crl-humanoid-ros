"""Dummy ability for demonstration purposes"""
import time
import limxsdk.robot.Rate as Rate
from limxsdk.ability.base_ability import BaseAbility
from limxsdk.ability.registry import register_ability

@register_ability("dummy/ability1")
class DummyAbility(BaseAbility):
    """A simple dummy ability that does nothing but log messages"""
    
    def initialize(self, config):
        """Initialize the dummy ability"""
        self.robot = self.get_robot_instance()
        self.update_rate = config.get("update_rate", 1.0) 
        self.logger.info(f"Dummy ability initialized with update_rate: {self.update_rate}Hz")
        return True
    
    def on_start(self):
        """Called when the ability starts"""
        self.logger.info("Dummy ability started")
    
    def on_main(self):
        """Main control loop"""
        rate = Rate(self.update_rate)
        while self.running:
            self.logger.info(f"{self.name} - Dummy running (timestamp: {time.time():.2f})")
            rate.sleep()
    
    def on_stop(self):
        """Called when the ability stops"""
        self.logger.info("Dummy ability stopped")