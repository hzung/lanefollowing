from PIL import Image, ImageDraw, ImageFont
from webcolors import hex_to_rgb
import json
import sys


def generate_img(background_img, boxes):
    background_img_path = background_img
    background_img = Image.open(background_img_path).convert("RGBA")
    text_img = Image.new('RGBA', background_img.size, color=(255, 255, 255, 0))
    draw = ImageDraw.Draw(text_img)

    for box in boxes:
        x = box.get("x")
        y = box.get("y")
        w = box.get("width")
        h = box.get("height")
        x = x - 0.5 * w
        y = y - 0.5 * h
        shape = [(x, y), (x + w, y + h)] 
        draw.rectangle(shape, fill=None, outline ="green") 

    return Image.alpha_composite(background_img, text_img)


# Configurations
if len(sys.argv) != 3:
    print("python3 main.py <path/to/image> <path/to/output/file>")
else:
    msg_id = sys.argv[1]
    output_img_name = sys.argv[2]

    template_img = "{}.jpg".format(msg_id)
    text_configs = json.loads(open("{}.json".format(msg_id), "r").read())
    output_img = generate_img(template_img, text_configs)
    output_img.save(output_img_name, "PNG")
    print('Generated: {}'.format(output_img_name))
