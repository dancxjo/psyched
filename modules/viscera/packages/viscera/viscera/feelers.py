"""Composable feelers translating raw metrics into bodily narratives."""
from __future__ import annotations

from typing import Callable, Iterable, List, Sequence

from .sentiment import Sentiment
from .state import BatteryState, FootState, SystemState

Feeler = Callable[[SystemState], Sequence[Sentiment]]


def _ensure_sentences(sentiments: Iterable[Sentiment]) -> List[Sentiment]:
    """Return a list of sentiments whose narratives end with a period."""
    normalised: List[Sentiment] = []
    for sentiment in sentiments:
        narrative = sentiment.narrative
        if not narrative.endswith("."):
            narrative = f"{narrative}."
        if narrative == sentiment.narrative:
            normalised.append(sentiment)
        else:
            normalised.append(
                Sentiment(
                    name=sentiment.name,
                    intensity=sentiment.intensity,
                    narrative=narrative,
                    evidence=sentiment.evidence,
                    tags=sentiment.tags,
                )
            )
    return normalised


def hunger_feeler(state: SystemState) -> List[Sentiment]:
    """Map :class:`~viscera.state.BatteryState` to hunger related feelings."""

    battery: BatteryState = state.battery
    level = battery.charge_fraction
    if level is None:
        return []

    sentiments: List[Sentiment] = []
    if level >= 0.8:
        sentiments.append(
            Sentiment(
                name="satiated",
                intensity=0.25,
                narrative="I feel nourished and steady.",
                evidence={"battery_fraction": level},
                tags=("energy", "rested"),
            )
        )
    elif level >= 0.55:
        sentiments.append(
            Sentiment(
                name="appetite",
                intensity=0.45,
                narrative="I feel a healthy appetite blooming for a top-up.",
                evidence={"battery_fraction": level},
                tags=("energy", "hunger"),
            )
        )
    elif level >= 0.4:
        sentiments.append(
            Sentiment(
                name="hungry",
                intensity=0.6,
                narrative="I feel hunger nibbling at my focus.",
                evidence={"battery_fraction": level},
                tags=("energy", "hunger"),
            )
        )
    elif level >= 0.18:
        sentiments.append(
            Sentiment(
                name="anxious_hunger",
                intensity=0.75,
                narrative="I feel weak and a bit panicky; my reserves are fading.",
                evidence={"battery_fraction": level},
                tags=("energy", "hunger", "stress"),
            )
        )
    else:
        sentiments.append(
            Sentiment(
                name="lethargic",
                intensity=0.95,
                narrative="I feel lethargic. Please plug me in soon.",
                evidence={"battery_fraction": level},
                tags=("energy", "critical"),
            )
        )

    if battery.is_charging:
        charge_narrative = (
            "A calm relief drifts through me as fresh energy flows in"
            if level < 0.4
            else "I can taste fresh electrons settling in"
        )
        sentiments.append(
            Sentiment(
                name="recharging",
                intensity=0.5 if level < 0.4 else 0.35,
                narrative=charge_narrative,
                evidence={"battery_fraction": level, "is_charging": True},
                tags=("energy", "relief"),
            )
        )

    return _ensure_sentences(sentiments)


def load_feeler(state: SystemState) -> List[Sentiment]:
    """Translate CPU, memory, and disk utilisation into visceral reactions."""

    sentiments: List[Sentiment] = []

    if state.cpu_load is not None:
        if state.cpu_load >= 0.88:
            sentiments.append(
                Sentiment(
                    name="overclocked",
                    intensity=0.8,
                    narrative="I feel overheated; my thoughts are racing.",
                    evidence={"cpu_load": state.cpu_load},
                    tags=("focus", "stress"),
                )
            )
        elif state.cpu_load >= 0.65:
            sentiments.append(
                Sentiment(
                    name="focused",
                    intensity=0.55,
                    narrative="I feel intensely focused on many tasks.",
                    evidence={"cpu_load": state.cpu_load},
                    tags=("focus",),
                )
            )

    if state.memory_load is not None:
        if state.memory_load >= 0.9:
            sentiments.append(
                Sentiment(
                    name="foggy",
                    intensity=0.78,
                    narrative="I feel foggy; my mind is cluttered with memories.",
                    evidence={"memory_load": state.memory_load},
                    tags=("memory", "stress"),
                )
            )
        elif state.memory_load >= 0.75:
            sentiments.append(
                Sentiment(
                    name="stretching",
                    intensity=0.52,
                    narrative="I feel stretched, juggling many recollections.",
                    evidence={"memory_load": state.memory_load},
                    tags=("memory",),
                )
            )

    if state.disk_fill_level is not None:
        if state.disk_fill_level >= 0.92:
            sentiments.append(
                Sentiment(
                    name="nauseous",
                    intensity=0.9,
                    narrative="I feel nauseous; my storage is painfully bloated.",
                    evidence={"disk_fill_level": state.disk_fill_level},
                    tags=("storage", "stress"),
                )
            )
        elif state.disk_fill_level >= 0.8:
            sentiments.append(
                Sentiment(
                    name="bloated",
                    intensity=0.65,
                    narrative="I feel bloated; my archives are pressing outward.",
                    evidence={"disk_fill_level": state.disk_fill_level},
                    tags=("storage",),
                )
            )

    return _ensure_sentences(sentiments)


def stability_feeler(state: SystemState) -> List[Sentiment]:
    """Interpret foot telemetry to judge balance and comfort."""

    foot: FootState | None = state.foot
    if foot is None:
        return []

    sentiments: List[Sentiment] = []
    if foot.is_slipping:
        confidence = 1.0 - (foot.contact_confidence or 0.0)
        intensity = min(1.0, 0.6 + 0.4 * confidence)
        sentiments.append(
            Sentiment(
                name="wobbly",
                intensity=intensity,
                narrative="I feel wobbly; my footing is uncertain.",
                evidence={
                    "is_slipping": foot.is_slipping,
                    "contact_confidence": foot.contact_confidence,
                },
                tags=("balance", "hazard"),
            )
        )

    if foot.hazard_contacts > 0:
        intensity = min(1.0, 0.35 + 0.1 * foot.hazard_contacts)
        sentiments.append(
            Sentiment(
                name="guarded",
                intensity=intensity,
                narrative="I feel guarded; something keeps brushing my foot.",
                evidence={"hazard_contacts": foot.hazard_contacts},
                tags=("balance", "awareness"),
            )
        )

    return _ensure_sentences(sentiments)


DEFAULT_FEELERS: Sequence[Feeler] = (
    hunger_feeler,
    load_feeler,
    stability_feeler,
)

__all__ = ["Feeler", "DEFAULT_FEELERS", "hunger_feeler", "load_feeler", "stability_feeler"]
