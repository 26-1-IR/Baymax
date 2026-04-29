import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _env_path(var_name, *paths):
    entries = [p for p in os.environ.get(var_name, '').split(os.pathsep) if p]
    for path in paths:
        if path and path not in entries:
            entries.append(path)
    return os.pathsep.join(entries)


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _normalize_credential(value):
    value = str(value).strip().lower()
    if value in ('handicapped', 'disabled', 'accessible', '장애인'):
        return 'handicapped'
    return 'general'


def _pick_korean_font(tk_font):
    candidates = (
        'Noto Sans CJK KR',
        'Noto Sans KR',
        'NanumGothic',
        'Nanum Gothic',
        'UnDotum',
        'Malgun Gothic',
        'AppleGothic',
    )
    families = set(tk_font.families())
    for candidate in candidates:
        if candidate in families:
            return candidate
    return 'TkDefaultFont'


def _show_config_gui(default_credential, default_yolo_model):
    config = {
        'user_credential': _normalize_credential(default_credential),
        'yolo_model': default_yolo_model,
        'cancelled': False,
    }

    if os.name != 'nt' and not os.environ.get('DISPLAY'):
        print('[parking_sim] DISPLAY가 없어 설정 GUI를 건너뜁니다.')
        return config

    try:
        import tkinter as tk
        import tkinter.font as tk_font
        from tkinter import filedialog, ttk
    except Exception as exc:
        print(f'[parking_sim] tkinter를 사용할 수 없어 설정 GUI를 건너뜁니다: {exc}')
        return config

    root = tk.Tk()
    root.title('Autonomous Parking Setup')
    root.resizable(False, False)
    root.configure(bg='#f5f5f5')

    font_family = _pick_korean_font(tk_font)
    default_font = (font_family, 10)
    title_font = (font_family, 14, 'bold')
    button_font = (font_family, 10, 'bold')

    for font_name in (
        'TkDefaultFont',
        'TkTextFont',
        'TkMenuFont',
        'TkHeadingFont',
        'TkCaptionFont',
        'TkSmallCaptionFont',
        'TkIconFont',
        'TkTooltipFont',
    ):
        try:
            tk_font.nametofont(font_name).configure(family=font_family)
        except tk.TclError:
            pass

    credential_var = tk.StringVar(value=config['user_credential'])
    yolo_var = tk.StringVar(value=config['yolo_model'])

    style = ttk.Style(root)
    style.configure('TLabel', font=default_font)
    style.configure('TButton', font=default_font)
    style.configure('TRadiobutton', font=default_font)
    style.configure('TLabelframe.Label', font=default_font)
    style.configure('Title.TLabel', font=title_font)
    style.configure('Body.TLabel', font=default_font)
    style.configure('Primary.TButton', font=button_font)

    frame = ttk.Frame(root, padding=18)
    frame.grid(row=0, column=0, sticky='nsew')

    ttk.Label(frame, text='주차 실행 설정', style='Title.TLabel').grid(
        row=0, column=0, columnspan=3, sticky='w'
    )
    ttk.Label(
        frame,
        text='시뮬레이션을 시작하기 전에 슬롯 조건을 선택하세요.',
        style='Body.TLabel',
    ).grid(row=1, column=0, columnspan=3, sticky='w', pady=(4, 14))

    slot_group = ttk.LabelFrame(frame, text='주차 슬롯')
    slot_group.grid(row=2, column=0, columnspan=3, sticky='ew')
    slot_group.columnconfigure(0, weight=1)

    ttk.Radiobutton(
        slot_group,
        text='일반 슬롯에 주차',
        value='general',
        variable=credential_var,
    ).grid(row=0, column=0, sticky='w', padx=12, pady=(10, 2))
    ttk.Label(
        slot_group,
        text='장애인 전용 슬롯은 후보에서 제외합니다.',
    ).grid(row=1, column=0, sticky='w', padx=34, pady=(0, 10))

    ttk.Radiobutton(
        slot_group,
        text='장애인 전용 슬롯 우선',
        value='handicapped',
        variable=credential_var,
    ).grid(row=2, column=0, sticky='w', padx=12, pady=(0, 2))
    ttk.Label(
        slot_group,
        text='빈 장애인 전용 슬롯을 먼저 선택하고, 없으면 다른 빈 슬롯을 고릅니다.',
    ).grid(row=3, column=0, sticky='w', padx=34, pady=(0, 10))

    ttk.Label(frame, text='YOLOv8 모델 경로').grid(
        row=3, column=0, sticky='w', pady=(14, 4)
    )
    model_entry = ttk.Entry(frame, textvariable=yolo_var, width=42)
    model_entry.grid(row=4, column=0, columnspan=2, sticky='ew')

    def browse_model():
        path = filedialog.askopenfilename(
            title='YOLOv8 모델 선택',
            filetypes=(('PyTorch model', '*.pt'), ('All files', '*.*')),
        )
        if path:
            yolo_var.set(path)

    ttk.Button(frame, text='찾기', command=browse_model).grid(
        row=4, column=2, sticky='ew', padx=(8, 0)
    )
    ttk.Label(
        frame,
        text='비워두면 slot_metadata.yaml의 메타데이터를 사용합니다.',
    ).grid(row=5, column=0, columnspan=3, sticky='w', pady=(4, 14))

    button_row = ttk.Frame(frame)
    button_row.grid(row=6, column=0, columnspan=3, sticky='e')

    def start():
        config['user_credential'] = credential_var.get()
        config['yolo_model'] = yolo_var.get().strip()
        config['cancelled'] = False
        root.destroy()

    def cancel():
        config['cancelled'] = True
        root.destroy()

    ttk.Button(button_row, text='취소', command=cancel).grid(row=0, column=0, padx=(0, 8))
    ttk.Button(button_row, text='시작', style='Primary.TButton', command=start).grid(
        row=0, column=1
    )

    root.protocol('WM_DELETE_WINDOW', cancel)
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f'+{x}+{y}')
    root.mainloop()
    return config


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('autonomous_parking')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')
    models_path = os.path.join(pkg_dir, 'models')
    gazebo_model_path = '/usr/share/gazebo-11/models'
    gazebo_resource_path = '/usr/share/gazebo-11'

    user_credential = LaunchConfiguration('user_credential').perform(context)
    yolo_model = LaunchConfiguration('yolo_model').perform(context)
    show_config_gui = LaunchConfiguration('show_config_gui').perform(context)

    if _as_bool(show_config_gui):
        config = _show_config_gui(user_credential, yolo_model)
        if config['cancelled']:
            return [
                LogInfo(msg='Parking setup GUI cancelled. Launch stopped.'),
                Shutdown(reason='Parking setup cancelled'),
            ]
        user_credential = config['user_credential']
        yolo_model = config['yolo_model']

    yolo_msg = yolo_model if yolo_model else 'YAML metadata fallback'

    return [
        LogInfo(
            msg=(
                f'Parking config: user_credential={user_credential}, '
                f'yolo_model={yolo_msg}'
            )
        ),

        # Gazebo model & resource paths
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            _env_path('GAZEBO_MODEL_PATH', gazebo_model_path, models_path),
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            _env_path('GAZEBO_RESOURCE_PATH', gazebo_resource_path, pkg_dir),
        ),

        # Gazebo with parking lot world (ego_hatchback included in world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # 1. Vision node — camera + YOLO (falls back to YAML if no model)
        Node(
            package='autonomous_parking',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yolo_model': yolo_model,
            }],
        ),

        # 2. Decision node — credential-based slot selection
        Node(
            package='autonomous_parking',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'user_credential': user_credential,
            }],
        ),

        # 3. Parking controller — observation points + parking execution
        Node(
            package='autonomous_parking',
            executable='parking_node',
            name='parking_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]


def generate_launch_description():
    # user_credential: 'general' (default) or 'handicapped'
    credential_arg = DeclareLaunchArgument(
        'user_credential',
        default_value='general',
        description='User parking credential: general | handicapped',
    )

    # Optional YOLOv8 model path (leave empty to use YAML ground truth)
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='',
        description='Path to YOLOv8 .pt model file (empty = yaml fallback)',
    )

    show_config_gui_arg = DeclareLaunchArgument(
        'show_config_gui',
        default_value='true',
        description='Show startup settings GUI before launching simulation',
    )

    return LaunchDescription([
        credential_arg,
        yolo_model_arg,
        show_config_gui_arg,
        OpaqueFunction(function=_launch_setup),
    ])
