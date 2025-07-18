from pydantic import BaseModel, Field
from typing import List, Literal, Annotated

# Define constrained float types using Annotated
XCoord = Annotated[float, Field(ge=-1000, le=1000)]
YCoord = Annotated[float, Field(ge=-1000, le=1000)]
Depth = Annotated[float, Field(ge=0, le=200)]
Tolerance = Annotated[float, Field(ge=0.1, le=10)]
TransitSpeed = Annotated[float, Field(ge=0.1, le=5.0)]
MainLight = Annotated[int, Field(ge=0, le=100)]

class NavigationTarget(BaseModel):
    x: XCoord = 0.0  # meters (relative to start)
    y: YCoord = 0.0
    depth: Depth = 10.0  # meters
    tolerance: Tolerance = 1.0  # meters
    action: Literal['none', 'inspect', 'sample', 'photograph', 'search'] = 'none'

class ElectricalConfig(BaseModel):
    main_light: MainLight = 70  # 0-100%
    uv_light: bool = False
    camera_mode: Literal['off', 'photo', 'video', 'scan'] = 'video'
    sonar_active: bool = True
    sensor_package: Literal['basic', 'full', 'environment'] = 'basic'

class VehicleConfig(BaseModel):
    target_sequence: List[NavigationTarget] = Field(
        default_factory=list,
        description="Ordered list of navigation targets"
    )
    electrical: ElectricalConfig = Field(
        default_factory=ElectricalConfig,
        description="Electrical system configuration"
    )
    transit_speed: TransitSpeed = 1.5  # m/s
    operation_mode: Literal['transit', 'search', 'inspection', 'manipulation'] = 'transit'
    replan_conditions: List[str] = Field(
        default_factory=lambda: ["obstacle_detected"],
        description="Events triggering replanning"
    )