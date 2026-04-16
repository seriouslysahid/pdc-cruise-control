from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True, slots=True)
class VehicleParams:
    m: float = 1600.0
    C_d: float = 0.32
    A: float = 2.4
    rho: float = 1.3
    C_rr: float = 0.01
    g: float = 9.8
    v0: float = 25.0
    u_min: float = 0.0
    u_max: float = 7000.0

    @property
    def b_eff(self) -> float:
        return self.rho * self.C_d * self.A * self.v0

    @property
    def tau(self) -> float:
        return self.m / self.b_eff

    @property
    def K(self) -> float:
        return 1.0 / self.b_eff

    @property
    def u0(self) -> float:
        return (
            0.5 * self.rho * self.C_d * self.A * self.v0**2
            + self.C_rr * self.m * self.g
        )

    @property
    def K_d(self) -> float:
        return self.g * self.m / self.b_eff

    def with_updates(self, **kwargs: float) -> "VehicleParams":
        return replace(self, **kwargs)


@dataclass(frozen=True, slots=True)
class ControllerParams:
    Kp: float
    tau_I: float | None = None
    tau_D: float | None = None
    N_filter: float = 10.0
    label: str = ""
    key: str = ""

    @property
    def has_integral(self) -> bool:
        return self.tau_I is not None

    @property
    def has_derivative(self) -> bool:
        return self.tau_D is not None
