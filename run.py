import json


def check_capacity(max_capacity: int, guests: list) -> bool:
    events = []

    for guest in guests:
        check_in = guest["check-in"]
        check_out = guest["check-out"]
        events.append((check_in, 1))
        events.append((check_out, -1))

    events.sort(key=lambda x: (x[0], x[1]))

    current = 0
    for i, change in events:
        current += change
        if current > max_capacity:
            return False

    return True


if __name__ == "__main__":
    max_capacity = int(input())
    n = int(input())

    guests = []
    for _ in range(n):
        guest_json = input()
        guest_data = json.loads(guest_json)
        guests.append(guest_data)

    result = check_capacity(max_capacity, guests)
    print(result)
