import json
import urllib.parse
from typing import Tuple, Optional
import requests
from requests.exceptions import RequestException


class UGV01Http:
    def __init__(self, ip_addr: str, timeout: float = 0.5):
        self.base_url = "http://" + ip_addr
        self.feedback_url = self.base_url + "/jsfb"
        self.timeout = timeout


    def send_json(self, payload: dict) -> Tuple[bool, Optional[str]]:
        """
        Send a JSON command over HTTP.
        Returns tuple (ok, response_text_or_error).
        ok == False when there in an HTTP error.
        """
        json_str = json.dumps(payload)
        encoded = urllib.parse.quote(json_str)
        url = f"{self.base_url}/js?json={encoded}"

        try:
            requests.get(url, timeout=self.timeout)
        
		# this request always gives an error
		# because the endpoint closes the connection without response
        except RequestException as e:
            pass
        
        try:
            response = requests.get(self.feedback_url)
            return True, response.text
        
        except RequestException as e:
            return False, str(e)
            
    #def get_feedback(self):
        

    def set_wheel_speeds(self, left_mps: float, right_mps: float) -> Tuple[bool, Optional[str]]:
        """
        CMD_SPEED_CTRL = 1.
        {"T":1,"L":<left m/s>,"R":<right m/s>}.
        the speed range is -0.5 ~ +0.5; positive value forward, negative value backward (from wiki).
        """
        cmd = {"T": 1, "L": left_mps, "R": right_mps}
        
        return self.send_json(cmd)
