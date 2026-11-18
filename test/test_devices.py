from pinky_sim_examples.devices import OFF, Lamp, LedStrip, check_emotion


def test_set_pixel_and_message_like_real_server():
    led = LedStrip(count=8)
    ok, msg = led.command('set_pixel', [4, 5, 6, 7], 0, 0, 255)
    assert ok and msg == 'Set pixel(s) [4, 5, 6, 7] to color (0, 0, 255).'
    assert led.pixels[:4] == [OFF] * 4 and led.pixels[4] == (0, 0, 255)


def test_fill_clear_and_unknown():
    led = LedStrip(count=4)
    assert led.command('FILL', [], 255, 0, 0)[0]
    assert led.pixels == [(255, 0, 0)] * 4
    assert led.command('clear', [], 0, 0, 0) == (True, 'Cleared all LEDs.')
    ok, msg = led.command('blink', [], 0, 0, 0)
    assert not ok and 'Unknown command' in msg


def test_out_of_range_pixel_changes_nothing():
    led = LedStrip(count=8)
    ok, msg = led.command('set_pixel', [1, 8], 255, 255, 255)
    assert not ok and msg.startswith('Failed:')
    assert led.pixels == [OFF] * 8


def test_brightness_scales_shown_color():
    led = LedStrip(count=1, max_brightness=100)
    led.command('fill', [], 200, 100, 0)
    assert led.set_brightness(50)[0]
    assert led.shown() == [(100, 50, 0)]
    assert not led.set_brightness(101)[0]


def test_lamp_on_off():
    lamp = Lamp()
    lamp.set(1.0, 0.0, 0.0, 1, 0, now_ms=0)
    assert lamp.color_at(123) == (255, 0, 0)
    lamp.set(1.0, 0.0, 0.0, 0, 0, now_ms=0)
    assert lamp.color_at(123) == OFF


def test_lamp_blink_half_period():
    lamp = Lamp()
    lamp.set(0.0, 1.0, 0.0, 2, 500, now_ms=1000)
    assert lamp.color_at(1200) == OFF            # 처음 500 ms 꺼짐 (실기와 같음)
    assert lamp.color_at(1700) == (0, 255, 0)
    assert lamp.color_at(2200) == OFF


def test_lamp_dimming_triangle():
    lamp = Lamp()
    lamp.set(0.0, 0.0, 1.0, 3, 1000, now_ms=0)
    assert lamp.color_at(0) == (0, 0, 0)
    assert lamp.color_at(500)[2] == 127
    assert lamp.color_at(1000) == (0, 0, 255)
    assert lamp.color_at(1500)[2] == 127


def test_lamp_rejects_bad_mode():
    assert not Lamp().set(1, 1, 1, 7, 100, now_ms=0)[0]
    assert not Lamp().set(1, 1, 1, 2, 0, now_ms=0)[0]


def test_emotion_names():
    assert check_emotion('happy') == (True, 'Emotion set to happy')
    assert not check_emotion('smile')[0]
