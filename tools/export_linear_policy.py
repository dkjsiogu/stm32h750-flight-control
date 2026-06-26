#!/usr/bin/env python3
"""导出 StaticMlpPolicy 可直接执行的 128-128 历史姿态策略。"""

from __future__ import annotations

from pathlib import Path


INPUT_DIM = 36
HIDDEN = 128
OUTPUT = 3
FRAME_DIM = 9
HISTORY = 4
PARAM_COUNT = 70
PARAM_PATH = Path("tools/static_policy_params.txt")


DEFAULT_PARAMS = [
    5.86136,
    0.806418,
    -4.40802,
    -2.08314,
    5.41356,
    -0.663206,
    -5.68171,
    -0.0286817,
    0.0489095,
    12.9824,
    1.22895,
    -3.17230,
    -2.02061,
    5.98654,
    -0.582643,
    -4.12559,
    -0.0427179,
    0.0417726,
    12.3348,
    0.818412,
    -3.86000,
    -3.51296,
    4.56577,
    -0.406924,
    -3.05539,
    0.00586839,
    -0.0431760,
    1.21296,
]


def input_index(frame: int, value: int) -> int:
    """计算历史窗口展开后的输入下标。"""
    return frame * FRAME_DIM + value


def load_params() -> list[float]:
    """读取搜索器写出的策略参数。"""
    if not PARAM_PATH.exists():
        return DEFAULT_PARAMS + [0.0] * (PARAM_COUNT - len(DEFAULT_PARAMS))
    values: list[float] = []
    for line in PARAM_PATH.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        values.extend(float(token) for token in line.split())
    if len(values) < PARAM_COUNT:
        values.extend([0.0] * (PARAM_COUNT - len(values)))
    return values[:PARAM_COUNT]


def add_relu_feature(
    *,
    slot: int,
    input_index_value: int,
    sign: float,
    threshold: float,
    layer1_weight: list[float],
    layer1_bias: list[float],
    layer2_weight: list[float],
) -> int:
    """添加一个 ReLU 特征并返回第二隐藏层槽位。"""
    layer1_weight[slot * INPUT_DIM + input_index_value] = sign
    layer1_bias[slot] = -threshold
    layer2_weight[slot * HIDDEN + slot] = 1.0
    return slot


def add_abs_feature(
    *,
    slot: int,
    input_index_value: int,
    threshold: float,
    layer1_weight: list[float],
    layer1_bias: list[float],
    layer2_weight: list[float],
) -> tuple[int, int]:
    """用两个 ReLU 单元表示带死区的绝对值特征。"""
    positive = add_relu_feature(
        slot=slot,
        input_index_value=input_index_value,
        sign=1.0,
        threshold=threshold,
        layer1_weight=layer1_weight,
        layer1_bias=layer1_bias,
        layer2_weight=layer2_weight,
    )
    negative = add_relu_feature(
        slot=slot + 1,
        input_index_value=input_index_value,
        sign=-1.0,
        threshold=threshold,
        layer1_weight=layer1_weight,
        layer1_bias=layer1_bias,
        layer2_weight=layer2_weight,
    )
    return positive, negative


def fmt(value: float) -> str:
    """格式化 C++ float 字面量。"""
    if abs(value) < 5.0e-9:
        value = 0.0
    text = f"{value:.9g}"
    if "e" not in text and "." not in text:
        text += ".0"
    return f"{text}f"


def format_array(values: list[float], indent: str = "        ") -> str:
    """按固定列宽格式化数组。"""
    lines: list[str] = []
    for start in range(0, len(values), 8):
        chunk = values[start : start + 8]
        lines.append(indent + ", ".join(fmt(value) for value in chunk) + ",")
    return "\n".join(lines)


def build_weights(params: list[float]) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float]]:
    """把策略参数展开成 128-128 MLP 权重。"""
    layer1_weight = [0.0] * (HIDDEN * INPUT_DIM)
    layer1_bias = [0.0] * HIDDEN
    layer2_weight = [0.0] * (HIDDEN * HIDDEN)
    layer2_bias = [0.0] * HIDDEN
    output_weight = [0.0] * (OUTPUT * HIDDEN)
    output_bias = [0.0] * OUTPUT

    selected_inputs = sorted(
        {
            input_index(frame, axis)
            for axis in range(3)
            for frame in (HISTORY - 3, HISTORY - 2, HISTORY - 1)
        }
        | {
            input_index(frame, axis + 3)
            for axis in range(3)
            for frame in (HISTORY - 2, HISTORY - 1)
        }
        | {input_index(HISTORY - 1, axis + 6) for axis in range(3)}
    )
    relu_slots = {index: pair for pair, index in enumerate(selected_inputs)}
    for index, pair in relu_slots.items():
        positive = pair * 2
        negative = positive + 1
        layer1_weight[positive * INPUT_DIM + index] = 1.0
        layer1_weight[negative * INPUT_DIM + index] = -1.0
        layer2_weight[positive * HIDDEN + positive] = 1.0
        layer2_weight[negative * HIDDEN + negative] = 1.0

    scale = params[27]
    for axis in range(3):
        offset = axis * 9
        p = params[offset : offset + 9]
        coeffs = {
            input_index(HISTORY - 1, axis): -p[0] - p[2] - p[3],
            input_index(HISTORY - 2, axis): p[2] + 2.0 * p[3] + p[6],
            input_index(HISTORY - 3, axis): -p[3],
            input_index(HISTORY - 1, axis + 3): -p[1] - p[4],
            input_index(HISTORY - 2, axis + 3): p[4],
            input_index(HISTORY - 1, axis + 6): -p[5],
        }
        output_bias[axis] = scale * p[7] + p[8]
        for index, coeff in coeffs.items():
            pair = relu_slots[index]
            positive = pair * 2
            negative = positive + 1
            output_weight[axis * HIDDEN + positive] += scale * coeff
            output_weight[axis * HIDDEN + negative] -= scale * coeff

    # 槽位 36 到 71 用于每轴 6 个带阈值的分段 ReLU 对，共 36 个隐藏单元。
    slot = 36
    for axis in range(3):
        gate_offset = 28 + axis * 14
        error_threshold = params[gate_offset + 0]
        omega_threshold = params[gate_offset + 1]
        prev_action_threshold = params[gate_offset + 2]
        error_now = input_index(HISTORY - 1, axis)
        omega_now = input_index(HISTORY - 1, axis + 3)
        previous_action = input_index(HISTORY - 1, axis + 6)

        feature_specs = (
            (error_now, error_threshold, params[gate_offset + 3]),
            (omega_now, omega_threshold, params[gate_offset + 4]),
            (previous_action, prev_action_threshold, params[gate_offset + 5]),
            (error_now, params[gate_offset + 6], params[gate_offset + 7]),
            (omega_now, params[gate_offset + 8], params[gate_offset + 9]),
            (previous_action, params[gate_offset + 10], params[gate_offset + 11]),
        )
        for feature_index, threshold, gain in feature_specs:
            positive, negative = add_abs_feature(
                slot=slot,
                input_index_value=feature_index,
                threshold=max(0.0, threshold),
                layer1_weight=layer1_weight,
                layer1_bias=layer1_bias,
                layer2_weight=layer2_weight,
            )
            output_weight[axis * HIDDEN + positive] += params[69] * gain
            output_weight[axis * HIDDEN + negative] += params[69] * gain
            slot += 2
        output_bias[axis] += params[69] * params[gate_offset + 12]

    return layer1_weight, layer1_bias, layer2_weight, layer2_bias, output_weight, output_bias


def main() -> None:
    """生成 src/model/generated_policy.cpp。"""
    weights = build_weights(load_params())
    layer1_weight, layer1_bias, layer2_weight, layer2_bias, output_weight, output_bias = weights

    content = f"""#include \"flight_control/model/generated_policy.hpp\"

namespace flight_control {{

namespace {{

const StaticMlpPolicyWeights kGeneratedWeights{{
    {{
{format_array(layer1_weight)}
    }},
    {{
{format_array(layer1_bias)}
    }},
    {{
{format_array(layer2_weight)}
    }},
    {{
{format_array(layer2_bias)}
    }},
    {{
{format_array(output_weight)}
    }},
    {{
{format_array(output_bias)}
    }},
}};

}}  // namespace

std::shared_ptr<const StaticMlpPolicyWeights> make_generated_policy_weights() {{
    return std::shared_ptr<const StaticMlpPolicyWeights>(&kGeneratedWeights, [](const StaticMlpPolicyWeights*) {{}});
}}

ModelAdapterConfig generated_model_config() {{
    ModelAdapterConfig config{{}};
    config.torque_limit_nm = 0.15f;
    config.target_reset_threshold = 2.0f;
    config.normalization.omega_mean = {{0.0f, 0.0f, 0.0f}};
    config.normalization.omega_std = {{1.0f, 1.0f, 1.0f}};
    return config;
}}

}}  // namespace flight_control
"""
    Path("src/model/generated_policy.cpp").write_text(content, encoding="utf-8")


if __name__ == "__main__":
    main()
