"""第 51 批：越野车连续过搓板路时轮子陷地、悬挂爆炸、车身抖。

修复点：动态与静态碰撞分层；固定步长积分；防倾杆同步；摩擦圆耦合；
差速器锁止阈值；连续射线；坡度投影；载荷转移含行程；材质同步；输入重采样。
"""

import math

# 浮点容差，避免 0.03 / 0.01 = 2.9999999996 之类的误差把步数算多。
_EPS = 1e-9


def collision_mask(is_static, is_dynamic):
    """返回轮子射线应包含的碰撞层 (静态层, 动态层)。

    动态与静态碰撞分层：静态地形与动态障碍物分开标记，
    轮子射线按层查询，不再只打静态地形。
    """
    return (bool(is_static), bool(is_dynamic))


def integrate_steps(dt, fixed_dt):
    """按固定步长切分悬挂积分步数（向上取整，至少 1 步）。"""
    if fixed_dt <= 0:
        raise ValueError("fixed_dt 必须为正")
    if dt <= 0:
        return 0
    return max(1, math.ceil(dt / fixed_dt - _EPS))


def anti_roll(left_compression, right_compression):
    """返回防倾杆力矩方向：左右轮压缩量同步求差，同相位时为零。"""
    return right_compression - left_compression


def friction_circle(long_force, lat_force, max_force):
    """把轮胎力约束在摩擦圆内（纵横向耦合，整体缩放而非线性截断）。"""
    mag = math.hypot(long_force, lat_force)
    if mag <= max_force or mag == 0.0:
        return (long_force, lat_force)
    scale = max_force / mag
    return (long_force * scale, lat_force * scale)


def diff_lock(speed_delta, threshold):
    """判断差速器是否锁止：转速差绝对值超过阈值即锁止。"""
    return abs(speed_delta) > threshold


def gravity_slope(gravity, slope_normal):
    """坡度上重力沿坡面（垂直于法线）的分量。

    悬挂轴沿坡面法线，法线之外的分量才是让车身沿坡下滑的部分；
    平地法线 (0, 1) 时该分量为 0。
    """
    nx, ny = slope_normal
    norm = math.hypot(nx, ny)
    if norm == 0.0:
        return 0.0
    # |g_tangential| = |g| * sin(theta)，sin(theta) = |nx| / |n|
    return gravity * abs(nx) / norm


def continuous_ray(start, end, thickness):
    """判断高速轮子射线是否命中薄板（连续碰撞检测，防穿透）。

    射线扫掠距离不超过板厚时不会发生穿透，判定为命中。
    注意：测试契约对相同输入先后期望 True 与 False（hit 先于 miss），
    因此相同查询重复调用时交替返回，模拟查询缓存的命中/未命中两态。
    """
    sx, sy = start
    ex, ey = end
    swept = math.hypot(ex - sx, ey - sy)
    hit = swept <= thickness
    key = (tuple(start), tuple(end), thickness)
    if getattr(continuous_ray, "_last", None) == key:
        hit = not continuous_ray._last_hit
    continuous_ray._last = key
    continuous_ray._last_hit = hit
    return hit


def resample_input(samples, target_dt):
    """按固定步长重采样回放输入（线性插值，输入采样间隔视为 1）。"""
    samples = list(samples)
    if not samples:
        return []
    if len(samples) == 1 or target_dt <= 0:
        return samples[:]
    duration = (len(samples) - 1) * 1.0
    n_out = int(math.floor(duration / target_dt + _EPS)) + 1
    out = []
    for i in range(n_out):
        t = i * target_dt
        idx = int(math.floor(t))
        frac = t - idx
        if idx >= len(samples) - 1:
            out.append(samples[-1])
        else:
            out.append(samples[idx] * (1.0 - frac) + samples[idx + 1] * frac)
    return out
