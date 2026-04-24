import numpy as np
import flight.pathfinding.utils.mask_gen as maskGen
import flight.pathfinding.path_subdivision as pathSub
from PIL import Image, ImageDraw


class SightTracker:
    # Initializes the field as a greyscale image with pixel size equating to the field's size and the
    # stored int value equating to the "confidence"/"known" value
    def __init__(self, field_size: tuple[int, int]):
        self.field_size = field_size
        self.map = np.array(Image.new("L", field_size, 0))

    def note_field_borders(
        self,
        field_borders: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]],
    ):
        pic_shape = Image.new("L", self.field_size, 255)
        ImageDraw.Draw(pic_shape).polygon(field_borders, outline=0, fill=0)
        self.map += np.array(pic_shape)

    # Takes the corner coords from a taken picture to increment the "confidence value"
    def note_pic(self, corner_coords: tuple[int, int, int, int]):
        picShape = Image.new("L", self.field_size, 0)
        ImageDraw.Draw(picShape).polygon(corner_coords, outline=1, fill=1)
        self.map += np.array(picShape)

    # Checks for holes within the sightTracker map, within the predicted photo area, and within the given path clearence
    # a True means a there is a hole and a False means theres no hole
    def check_holes(
        self, proto_photo: maskGen.PolygonMask, path_seg: maskGen.PolygonMask
    ):  # Maybe include the previous and following path segments?
        for i in range(max(proto_photo.bottom_x, 0), min(proto_photo.top_x, self.field_size[0])):
            for j in range(
                max(proto_photo.bottom_y, 0), min(proto_photo.top_y, self.field_size[1])
            ):
                if (
                    self.map[i][j] == 0
                    and proto_photo.body[i - proto_photo.bottom_x][j - proto_photo.bottom_y] > 0
                    and path_seg.body[i - path_seg.bottom_x][j - path_seg.bottom_y] > 0
                ):
                    return True
        return False

    def save_tracker_as_image(self):
        im = Image.fromarray(self.map)
        im.save("latest_test_seen_tracker_save.jpeg")


# Removes coords that are redundant to check
def remove_extra_coords(
    seen: SightTracker,
    goto_points: tuple[tuple[float, float]],
    data_sup: tuple[tuple[pathSub.Node, pathSub.Node, bool]],
    cam_size: tuple[float, float],
):
    updated_goto_points: tuple[tuple[float, float]] = []
    for i in range(len(goto_points)):
        if data_sup[i][2] == True:
            # Segment is a pie slice type
            seg_mask = maskGen.PolygonMask(data_sup[i][0], data_sup[i][1], True)
            tan_angle = np.arctan(
                (goto_points[i][1] - data_sup[i][0].parentMine.y)
                / (goto_points[i][0] - data_sup[i][0].parentMine.x)
            ) + (np.pi / 2)
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)
        else:
            # Segment is a straight type
            seg_mask = maskGen.PolygonMask(data_sup[i][0], data_sup[i][1])
            tan_angle = np.arctan(
                (data_sup[i][0].y - data_sup[i][1].y) / (data_sup[i][0].x - data_sup[i][1].x)
            )
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)

        if seen.check_holes(proto_photo, seg_mask):
            updated_goto_points.append(goto_points[i])

    return updated_goto_points


def remove_extra_coords(
    seen: SightTracker,
    goto_points: tuple[tuple[float, float]],
    data_sup: tuple[tuple[pathSub.Node, pathSub.Node, bool]],
    cam_size: tuple[float, float],
    test: bool,
):
    updated_goto_points: tuple[tuple[float, float]] = []
    seen.save_tracker_as_image()
    seg_image = Image.fromarray(seen.map)
    predict_pic_image = Image.fromarray(seen.map)
    next_color = [(239, 55, 44), (88, 96, 195), (255, 226, 105)]
    for i in range(len(goto_points)):
        if data_sup[i][2] == True:
            # Segment is a pie slice type
            seg_mask = maskGen.PolygonMask(data_sup[i][0], data_sup[i][1], True)

            # Image test case generation
            temp_array = np.array(
                Image.new(
                    "RGB", [seg_mask.top_x - seg_mask.bottom_x, seg_mask.top_y - seg_mask.bottom_y]
                )
            )
            temp_array[seg_mask.body] = (161, 126, 172)
            temp_image = Image.fromarray(temp_array)
            seg_image.paste(temp_image, (seg_mask.bottom_x, seg_mask.top_y))

            # Code resumes as normal
            tan_angle = np.arctan(
                (goto_points[i][1] - data_sup[i][0].parentMine.y)
                / (goto_points[i][0] - data_sup[i][0].parentMine.x)
            ) + (np.pi / 2)
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)
        else:
            # Segment is a straight type
            seg_mask = maskGen.PolygonMask(data_sup[i][0], data_sup[i][1])

            # Image test case generation
            temp_array = np.array(
                Image.new(
                    "RGB", [seg_mask.top_x - seg_mask.bottom_x, seg_mask.top_y - seg_mask.bottom_y]
                )
            )
            temp_array[seg_mask.body] = (239, 146, 157)
            temp_image = Image.fromarray(temp_array)
            seg_image.paste(temp_image, (seg_mask.bottom_x, seg_mask.top_y))

            # Image
            tan_angle = np.arctan(
                (data_sup[i][0].y - data_sup[i][1].y) / (data_sup[i][0].x - data_sup[i][1].x)
            )
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)

        # Image test case generation
        temp_array = np.array(
            Image.new(
                "RGB",
                [
                    proto_photo.top_x - proto_photo.bottom_x,
                    proto_photo.top_y - proto_photo.bottom_y,
                ],
            )
        )
        temp_array[seg_mask.body] = next_color[(i % 3)]
        temp_image = Image.fromarray(temp_array)
        predict_pic_image.paste(
            temp_image,
            (proto_photo.bottom_x, proto_photo.top_y, proto_photo.top_x, proto_photo.bottom_y),
        )

        if seen.check_holes(proto_photo, seg_mask):
            updated_goto_points.append(goto_points[i])

    seg_image.save("latest_test_seg_masks.jpeg")
    predict_pic_image.save("latest_test_prediction_masks.jpeg")
    return updated_goto_points


if __name__ == "__main__":
    # Screw My stupid chud life I can't test this until the field is working again.
    pass
