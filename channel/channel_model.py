"""
CSU33D03 Main Project - LEO Satellite Channel Model
Group 7-9

Simulates the realistic physical and environmental characteristics
of a LEO satellite communication link between an offshore wind turbine
and a space station.

Key parameters modelled:
  - Propagation delay  (LEO altitude ~550 km → ~1.8 ms one-way; but geometry
                        means slant-range varies → 3–20 ms one-way modelled)
  - Round-trip time    (RTT = 2 × one-way + processing)
  - Packet loss        (rain fade, free-space path loss, burst errors)
  - Contact windows    (satellite only visible for ~8–12 min per pass)
  - Doppler shift      (modelled as jitter on delay)
  - Ocean wave motion  (adds ±pointing error → additional loss probability)
"""

import random
import time
import math
import threading
import logging

log = logging.getLogger('channel')
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s %(message)s'
)


# ── Physical constants & defaults ──────────────────────────────────────────────
LEO_ALTITUDE_KM      = 550          # Starlink-style LEO
EARTH_RADIUS_KM      = 6371
SPEED_OF_LIGHT_KMS   = 299_792     # km/s
MIN_ELEVATION_DEG    = 10          # satellite not usable below 10° elevation

# Slant range at min elevation (km)
# slant_range = sqrt(R² sin²θ + 2Rh + h²) − R sinθ  (approx)
def slant_range_km(elevation_deg: float) -> float:
    theta = math.radians(elevation_deg)
    R = EARTH_RADIUS_KM
    h = LEO_ALTITUDE_KM
    return math.sqrt((R * math.sin(theta))**2 + 2*R*h + h**2) - R * math.sin(theta)


MAX_SLANT_KM = slant_range_km(MIN_ELEVATION_DEG)   # worst-case range
MIN_SLANT_KM = LEO_ALTITUDE_KM                      # directly overhead

ONE_WAY_MIN_MS = (MIN_SLANT_KM / SPEED_OF_LIGHT_KMS) * 1000   # ~1.84 ms
ONE_WAY_MAX_MS = (MAX_SLANT_KM / SPEED_OF_LIGHT_KMS) * 1000   # ~8.8 ms at 10°


# ── Channel Model ──────────────────────────────────────────────────────────────
class SatelliteChannelModel:
    """
    Emulates the LEO satellite communication channel.

    Usage:
        channel = SatelliteChannelModel()
        channel.start()                         # starts orbital simulation
        delay, lost = channel.transmit(packet)  # returns (delay_s, was_lost)
    """

    def __init__(
        self,
        contact_window_s   = 600,   # 10-minute pass
        gap_between_pass_s = 5400,  # ~90-min orbit; ~10-min visible per pass
        base_loss_rate     = 0.02,  # 2% baseline packet loss
        ocean_swell_factor = 0.5,   # 0–1, adds pointing error
    ):
        self.contact_window_s    = contact_window_s
        self.gap_between_pass_s  = gap_between_pass_s
        self.base_loss_rate      = base_loss_rate
        self.ocean_swell_factor  = ocean_swell_factor

        # Orbital state
        self._in_contact   = True
        self._pass_start   = time.time()
        self._lock         = threading.Lock()
        self._running      = False
        self._thread       = None

        # Stats
        self.total_sent = 0
        self.total_lost = 0
        self.total_delayed_ms = 0.0

    # ── Orbital pass simulation ────────────────────────────────────────────
    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._orbital_loop, daemon=True)
        self._thread.start()
        log.info("Channel model started. First contact window open.")

    def stop(self):
        self._running = False

    def _orbital_loop(self):
        """Cycles satellite contact windows in real time (compressed scale)."""
        while self._running:
            # Contact window open
            with self._lock:
                self._in_contact = True
                self._pass_start = time.time()
            log.info(
                f"[CHANNEL] Satellite contact window OPEN "
                f"({self.contact_window_s}s pass)"
            )
            time.sleep(self.contact_window_s)

            # Gap between passes
            with self._lock:
                self._in_contact = False
            log.warning(
                f"[CHANNEL] Satellite contact window CLOSED. "
                f"Next pass in {self.gap_between_pass_s}s."
            )
            time.sleep(self.gap_between_pass_s)

    # ── Elevation angle during pass ────────────────────────────────────────
    def _current_elevation_deg(self) -> float:
        """
        Approximate elevation angle based on time into pass.
        Models a sinusoidal rise-and-set over the contact window.
        """
        with self._lock:
            elapsed = time.time() - self._pass_start

        frac = min(elapsed / self.contact_window_s, 1.0)
        # sine curve: starts low, peaks at halfway, drops back
        elev = MIN_ELEVATION_DEG + (90 - MIN_ELEVATION_DEG) * math.sin(math.pi * frac)
        return elev

    # ── One-way propagation delay ──────────────────────────────────────────
    def _propagation_delay_ms(self) -> float:
        elev   = self._current_elevation_deg()
        slant  = slant_range_km(elev)
        base_delay = (slant / SPEED_OF_LIGHT_KMS) * 1000   # ms

        # Doppler / atmospheric jitter ±10%
        jitter = base_delay * 0.10 * (random.random() * 2 - 1)
        return base_delay + jitter

    # ── Packet loss probability ────────────────────────────────────────────
    def _loss_probability(self) -> float:
        elev = self._current_elevation_deg()

        # Path loss increases near horizon (low elevation)
        elevation_loss = max(0, (30 - elev) / 30) * 0.15  # up to 15% extra at 10°

        # Ocean swell: turbine pitches → antenna misalignment
        swell_loss = self.ocean_swell_factor * 0.05 * random.random()

        # Occasional burst fade (rain, ionosphere)
        burst_fade = 0.20 if random.random() < 0.03 else 0.0  # 3% chance of fade event

        return min(self.base_loss_rate + elevation_loss + swell_loss + burst_fade, 1.0)

    # ── Main transmit interface ────────────────────────────────────────────
    def transmit(self, packet: bytes) -> tuple[float, bool]:
        """
        Simulate sending a packet through the LEO channel.

        Returns:
            (delay_seconds, was_lost)
            - delay_seconds: float  — how long to wait before delivery
            - was_lost: bool        — True if packet dropped

        The caller is responsible for sleeping(delay_seconds) and deciding
        how to handle loss (retransmit, etc).
        """
        with self._lock:
            in_contact = self._in_contact

        self.total_sent += 1

        if not in_contact:
            log.warning("[CHANNEL] No satellite contact — packet dropped.")
            self.total_lost += 1
            return 0.0, True

        loss_prob = self._loss_probability()
        if random.random() < loss_prob:
            log.debug(f"[CHANNEL] Packet lost (loss_prob={loss_prob:.3f})")
            self.total_lost += 1
            return 0.0, True

        one_way_ms  = self._propagation_delay_ms()
        rtt_ms      = one_way_ms * 2           # one-way × 2 for round trip
        delay_s     = rtt_ms / 1000.0

        self.total_delayed_ms += rtt_ms
        log.debug(
            f"[CHANNEL] Packet delivered. "
            f"Elev={self._current_elevation_deg():.1f}° "
            f"delay={rtt_ms:.2f}ms"
        )
        return delay_s, False

    # ── One-way delay (for display / logging) ─────────────────────────────
    def one_way_delay_ms(self) -> float:
        return self._propagation_delay_ms()

    # ── Status ─────────────────────────────────────────────────────────────
    def status(self) -> dict:
        with self._lock:
            in_contact = self._in_contact
        loss_rate = self.total_lost / max(self.total_sent, 1)
        return {
            'in_contact'        : in_contact,
            'elevation_deg'     : round(self._current_elevation_deg(), 2) if in_contact else None,
            'one_way_delay_ms'  : round(self.one_way_delay_ms(), 3) if in_contact else None,
            'total_sent'        : self.total_sent,
            'total_lost'        : self.total_lost,
            'effective_loss_pct': round(loss_rate * 100, 2),
        }
