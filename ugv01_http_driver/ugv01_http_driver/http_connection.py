import json
import urllib.parse
from typing import Any, Tuple

import requests
from requests.exceptions import RequestException


class UGV01Http:
    def __init__(self, ip_addr: str, timeout: float = 0.25) -> None:
        self.base_url = "http://" + ip_addr
        self.feedback_url = self.base_url + "/jsfb"
        self.timeout = timeout


    def send_json(self, payload: dict[str, Any]) -> None:
        """
        Sends a JSON command over HTTP.
        """
        
        json_str = json.dumps(payload)
        encoded = urllib.parse.quote(json_str)
        url = f"{self.base_url}/js?json={encoded}"

        try:
            requests.get(url, timeout=self.timeout)
        
        # this request always gives an error
        # because the endpoint closes the connection without response
        except RequestException:
            pass


    def get_feedback(self) -> Tuple[bool, dict[str, Any] | str]:
        """
        Polls the robot's feedback endpoint.
        
        Returns
        -------
        Tuple[bool, dict[str, Any] | str]
            The bool indicates the success of the operation.
            If True, the second element is the parsed json dictionary.
            If False, the second element is an error message.
        """
        
        try:
            response = requests.get(self.feedback_url, timeout=self.timeout)
            response.raise_for_status()

            if not response.text.strip():
                return False, "Empty response from feedback endpoint"

            try:
                data = response.json()

                if isinstance(data, dict):
                    return True, data
                
                return False, f"Feedback JSON is not a dict: {type(data).__name__}"
            
            except ValueError:
                return False, f"Invalid JSON from feedback endpoint: {response.text!r}"

        except RequestException as e:
            return False, f"Feedback request failed: {e}"


    def send_generic_json(self, payload: dict[str, Any]) -> Tuple[bool, dict[str, Any] | str]:
        """
        Sends an arbitrary JSON command and then reads the robot feedback endpoint.

        Use this for request/response-style commands where some feedback may be expected.
        """
        
        self.send_json(payload)
        
        return self.get_feedback()


    def set_wheel_speeds(self, left_mps: float, right_mps: float) -> Tuple[bool, str]:
        """
        Sets the robot's wheels speeds to the given values.
        
        SPEED_INPUT = 1.
        {"T":1, "L":<left m/s>, "R":<right m/s>}.\n
        The speed range is -1 ~ +1; positive value forward, negative value backward.
        
        Parameters
        ----------
        left_mps : float
            the speed for the left wheel, in meters per second
        right_mps : float
            the speed for the right wheel, in meters per second
        
        Returns
        -------
        Tuple[bool, str]
            a bool to indicate the success of the operation and a string to provide more information
        """
        
        cmd = {"T": 1, "L": left_mps, "R": right_mps}
        self.send_json(cmd)
        
        return True, "set-wheel-speeds command sent"


    def get_ina219_info(self) -> Tuple[bool, dict[str, Any] | str]:
        cmd = {"T": 70}
        self.send_json(cmd)
        
        return self.get_feedback()


    def get_imu_info(self) -> Tuple[bool, dict[str, Any] | str]:
        cmd = {"T": 71}
        self.send_json(cmd)
        
        return self.get_feedback()


    def get_encoder_info(self) -> Tuple[bool, dict[str, Any] | str]:
        cmd = {"T": 73}
        self.send_json(cmd)

        return self.get_feedback()
    

    def get_device_info(self) -> Tuple[bool, dict[str, Any] | str]:
        url = f"{self.base_url}/deviceInfo"

        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()

            try:
                data = response.json()
            except ValueError:
                return False, f"Invalid JSON from deviceInfo endpoint: {response.text!r}"

            if isinstance(data, dict):
                return True, data

            return False, f"deviceInfo JSON is not a dict: {type(data).__name__}"

        except RequestException as e:
            return False, f"deviceInfo request failed: {e}"
