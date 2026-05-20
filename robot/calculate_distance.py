width = 870   # x
depth = 570   # y

upperleft = [-0.43329628205150544, -0.02144956443654847, -0.04050681359114769,
             -0.02845502221738299, -0.040604689123682154, -1.6054210794902326]

center = [0.013300045080078107, -0.28567798125382166, -0.04030660563747085,
          -0.014191398730641683, -0.039659007178467535, -1.593094934156127]


def calculate_coords(pixel_coords, upperleft, scales, z_value=None):
    px, py = pixel_coords[:2]

    wx = upperleft[0] + px * scales[0]
    wy = upperleft[1] + py * scales[1]
    wz = upperleft[2] if z_value is None else z_value

    return [wx, wy, wz]


if __name__ == "__main__":
    x_scale = (center[0] - upperleft[0]) / (width / 2)
    y_scale = (center[1] - upperleft[1]) / (depth / 2)

    scales = [x_scale, y_scale]

    result = calculate_coords([142.5, 217.5, 0.0], upperleft, scales)
    print(result)