import random

board = [[0] * 5 for _ in range(5)]
markers = [[False] * 5 for _ in range(5)]
on_board = set()
drawn = set()


def reset_game():
    start_num = 1
    stop_num = 15
    for x in range(5):
        for y in range(5):
            while True:
                num = random.randint(start_num, stop_num)
                if num not in on_board:
                    board[x][y] = num
                    on_board.add(num)
                    break
        start_num += 15
        stop_num += 15
    switch_rows_and_columns()
    print_board()


def print_board():
    for row in range(5):
        for col in range(5):
            if markers[row][col]:
                print("#", end=" ")
            else:
                print(str(board[row][col]), end=" ")

            if col == 4:
                print()


def switch_rows_and_columns():
    global board, markers
    new_board = [[0] * 5 for _ in range(5)]
    new_markers = [[False] * 5 for _ in range(5)]

    for i in range(5):
        for j in range(5):
            new_board[j][i] = board[i][j]
            new_markers[j][i] = markers[i][j]

    board = new_board
    markers = new_markers


def game_end():
    # check col
    for col in range(5):
        if markers[0][col]:
            for row in range(1, 5):
                if not markers[row][col]:
                    break
                elif row == 4:
                    return True
        else:
            break

    # check row
    for row in range(5):
        if markers[row][0]:
            for col in range(1, 5):
                if not markers[row][col]:
                    break
                elif row == 4:
                    return True
        else:
            break

    # check left down diag
    for row in range(5):
        if markers[row][row]:
            if row == 4:
                return True
        else:
            break

    # check right down diag
    for row in range(5):
        if markers[4 - row][4 - row]:
            if row == 4:
                return True
        else:
            break

    return False


def play_game():
    # find random number between 1-75
    # check to see if already drawn
    count = 0
    while not game_end():
        num = random.randint(0, 74)
        if num not in drawn:
            count += 1
            drawn.add(num)

            print("Random number drawn: " + str(num))

            if num in on_board:
                for row in range(5):
                    for col in range(5):
                        if board[row][col] == num:
                            markers[row][col] = True

            print_board()

    print("BINGO!")
    print("Total turns to win: " + str(count))


if __name__ == '__main__':
    reset_game()
    play_game()
