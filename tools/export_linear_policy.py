#!/usr/bin/env python3
"""导出可由 StaticMlpPolicy 精确执行的历史线性姿态策略。"""

from __future__ import annotations

from pathlib import Path


INPUT_DIM = 36
HIDDEN = 64
OUTPUT = 3
FRAME_DIM = 9
HISTORY = 4

# 每轴 9 个参数：姿态误差、角速度、误差一阶差分、误差二阶差分、
# 角速度差分、上一动作、上一帧误差、轴偏置、输出偏置；最后一项是全局输出比例。
PARAMS = [
    7.66965,
    1.00643,
    -4.10554,
    -2.31130,
    4.73164,
    -0.657288,
    -5.27284,
    -0.0162582,
    0.0228768,
    13.3204,
    1.25820,
    -3.38287,
    -1.66585,
    5.25498,
    -0.671618,
    -3.98049,
    -0.0378593,
    0.0294485,
    12.7505,
    0.82009,
    -4.16842,
    -3.49169,
    4.99741,
    -0.525458,
    -3.39140,
    -0.0289791,
    -0.0,
    1.08888,
]


def input_index(frame: int, value: int) -> int:
    """计算历史窗口展开后的输入下标。"""
    return frame * FRAME_DIM + value


def axis_coefficients(axis: int) -> tuple[dict[int, float], float]:
    """把搜索参数展开为当前 MLP 输出层前激活的线性系数。"""
    offset = axis * 9
    p = PARAMS[offset : offset + 9]
    scale = PARAMS[27]
    coeffs = {
        input_index(HISTORY - 1, axis): -p[0] - p[2] - p[3],
        input_index(HISTORY - 2, axis): p[2] + 2.0 * p[3] + p[6],
        input_index(HISTORY - 3, axis): -p[3],
        input_index(HISTORY - 1, axis + 3): -p[1] - p[4],
        input_index(HISTORY - 2, axis + 3): p[4],
        input_index(HISTORY - 1, axis + 6): -p[5],
    }
    return {key: scale * value for key, value in coeffs.items()}, scale * p[7] + p[8]


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


def main() -> None:
    """生成 src/model/generated_policy.cpp。"""
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

    layer1_weight = [0.0] * (HIDDEN * INPUT_DIM)
    layer1_bias = [0.0] * HIDDEN
    layer2_weight = [0.0] * (HIDDEN * HIDDEN)
    layer2_bias = [0.0] * HIDDEN
    output_weight = [0.0] * (OUTPUT * HIDDEN)
    output_bias = [0.0] * OUTPUT

    for index, pair in relu_slots.items():
        positive = pair * 2
        negative = positive + 1
        layer1_weight[positive * INPUT_DIM + index] = 1.0
        layer1_weight[negative * INPUT_DIM + index] = -1.0
        layer2_weight[positive * HIDDEN + positive] = 1.0
        layer2_weight[negative * HIDDEN + negative] = 1.0

    for axis in range(3):
        coeffs, bias = axis_coefficients(axis)
        output_bias[axis] = bias
        for index, coeff in coeffs.items():
            pair = relu_slots[index]
            positive = pair * 2
            negative = positive + 1
            output_weight[axis * HIDDEN + positive] += coeff
            output_weight[axis * HIDDEN + negative] -= coeff

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
