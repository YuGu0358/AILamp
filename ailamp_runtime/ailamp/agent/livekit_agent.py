from __future__ import annotations

from ailamp.config import load_hardware_config
from ailamp.paths import resolve_project_path
from ailamp.services.behavior import BehaviorService
from ailamp.services.led_serial import LEDSerialService
from ailamp.services.motor import MotorService
from ailamp.models import VisionEvent, VisionEventType


class AILampToolbox:
    def __init__(self, config_path: str):
        self.config = load_hardware_config(config_path)
        self.behavior = BehaviorService()
        self.last_event = VisionEvent(VisionEventType.NO_PERSON)
        self.led = LEDSerialService(self.config.led.port, self.config.led.count, self.config.led.baudrate)
        self.motors = MotorService(
            self.config.motors.port,
            self.config.system.project_name.lower(),
            resolve_project_path(self.config.simulation.recordings_dir),
        )
        self._outputs_connected = False

    def connect_outputs(self) -> None:
        if self._outputs_connected:
            return
        self.led.connect()
        self.motors.connect()
        self._outputs_connected = True

    def current_vision_state(self) -> str:
        return self.last_event.event_type.value

    def motion_for_current_vision(self) -> tuple[str, tuple[int, int, int]]:
        action = self.behavior.decide(self.last_event)
        return action.motion, action.rgb

    def play_recording(self, recording_name: str) -> str:
        self.connect_outputs()
        self.motors.play(recording_name)
        return f"playing {recording_name}"

    def set_light(self, red: int, green: int, blue: int) -> str:
        self.connect_outputs()
        return self.led.solid(red, green, blue)


def run_agent(config_path: str) -> None:
    # Import lazily so non-voice commands do not require LiveKit/OpenAI deps.
    from dotenv import load_dotenv  # type: ignore
    from livekit import agents  # type: ignore

    load_dotenv()
    toolbox = AILampToolbox(config_path)

    async def entrypoint(ctx):  # pragma: no cover - requires LiveKit runtime
        from livekit.agents import Agent, AgentSession, RoomInputOptions, function_tool  # type: ignore
        from livekit.plugins import noise_cancellation, openai  # type: ignore

        class AILampAgent(Agent):
            def __init__(self):
                super().__init__(
                    instructions=(
                        "You are AILamp, an interactive robotic desk lamp. "
                        "Use concise English. You can use motion and light tools to respond."
                    )
                )

            @function_tool
            async def get_vision_state(self) -> str:
                return toolbox.current_vision_state()

            @function_tool
            async def suggest_motion_for_vision(self) -> str:
                motion, rgb = toolbox.motion_for_current_vision()
                return f"motion={motion} rgb={rgb}"

            @function_tool
            async def play_recording(self, recording_name: str) -> str:
                return toolbox.play_recording(recording_name)

            @function_tool
            async def set_light(self, red: int, green: int, blue: int) -> str:
                return toolbox.set_light(red, green, blue)

        session = AgentSession(llm=openai.realtime.RealtimeModel(voice="ballad"))
        await session.start(
            room=ctx.room,
            agent=AILampAgent(),
            room_input_options=RoomInputOptions(noise_cancellation=noise_cancellation.BVC()),
        )

    agents.cli.run_app(agents.WorkerOptions(entrypoint_fnc=entrypoint, num_idle_processes=1))
