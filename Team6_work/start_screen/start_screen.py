import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
from PIL import Image, ImageTk
import fitz  # PyMuPDF (used for PDF rendering)

from Team6_work.image_description import ImageDescriber

class ReminiscenceTherapyGUI(ttk.Frame):
    def __init__(self, root, patient_info, theme_path="../Azure-ttk-theme-main/azure.tcl"):
        super().__init__(root, padding=20)
        self.root = root

        # Apply Azure ttk theme
        self.root.tk.call("source", theme_path)
        self.root.tk.call("set_theme", "light")

        # Make the window full screen and set minimum size
        self.root.title("Reminiscence Therapy GUI")
        self.root.state("zoomed")
        self.root.minsize(800, 600)
        self.pack(fill="both", expand=True)

        # Store basic patient data
        self.patient_name = str(patient_info.get("name", "Unknown"))
        self.patient_age = str(patient_info.get("age", "Unknown"))
        self.history_path = patient_info.get("history_path")

        # Variables to hold current image state
        self.image_path = None
        self.image_type = tk.StringVar(value="personal")  # 'personal' or 'generic'

        # Build the GUI layout
        self._build_left_image_panel()
        self._build_right_side_panel()

        # Allow layout to resize nicely
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2)
        self.rowconfigure(0, weight=1)

        # If history PDF is already set, show preview
        if self.history_path:
            self._preview_pdf(self.history_path)

        if self.image_path is not None:
            self.image_describer = ImageDescriber(image_path=self.image_path)

    def _build_left_image_panel(self):
        # Panel for uploading and showing image
        frame = ttk.LabelFrame(self, text="Reminiscence Image", padding=12)
        frame.grid(row=0, column=0, sticky="nsew", padx=(0, 20))

        self.image_label = ttk.Label(frame, text="No image uploaded", anchor="center", justify="center")
        self.image_label.pack(expand=True, fill="both")

        ttk.Button(frame, text="Upload Image", command=self._upload_image).pack(pady=8)

        # Add a new section for image description
        desc_frame = ttk.LabelFrame(frame, text="Image Description", padding=8)
        desc_frame.pack(fill="x", expand=False, pady=(10, 0))

        self.image_description = tk.Text(desc_frame, height=5, wrap="word", borderwidth=0, highlightthickness=0)
        self.image_description.pack(fill="both", expand=True)

    def _upload_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image Files", "*.png *.jpg *.jpeg")])
        if file_path:
            self.image_path = file_path
            image = Image.open(file_path)
            image.thumbnail((600, 600))
            photo = ImageTk.PhotoImage(image)
            self.image_label.configure(image=photo, text="", anchor="center", justify="center")
            self.image_label.image = photo  # keep reference

            self.image_describer = ImageDescriber(image_path=self.image_path)
            self.image_description.insert('1.0', self.image_describer.response.output_text)

    def _build_right_side_panel(self):
        # Right-hand panel for info and controls
        frame = ttk.Frame(self)
        frame.grid(row=0, column=1, sticky="nsew")

        # Patient info
        info = ttk.LabelFrame(frame, text="Patient Info", padding=12)
        info.pack(fill="x")
        ttk.Label(info, text=f"Name: {self.patient_name}").pack(anchor="w")
        ttk.Label(info, text=f"Age: {self.patient_age}").pack(anchor="w")

        self.history_label = ttk.Label(info, text="History: none uploaded", foreground="grey")
        self.history_label.pack(anchor="w", pady=(5, 0))
        ttk.Button(info, text="Upload History PDF", command=self._upload_pdf).pack(anchor="e", pady=5)

        # PDF preview area
        self.preview_box = ttk.LabelFrame(frame, text="History Preview", padding=6)
        self.preview_box.pack(fill="x", pady=10)

        self.preview_label = ttk.Label(self.preview_box, text="No preview available", anchor="center", justify="center")
        self.preview_label.pack(expand=True, fill="both")

        # Image type radio buttons
        img_type = ttk.LabelFrame(frame, text="Image Type", padding=12)
        img_type.pack(fill="x", pady=10)
        ttk.Radiobutton(img_type, text="Personal", variable=self.image_type, value="personal").pack(anchor="w")
        ttk.Radiobutton(img_type, text="Generic", variable=self.image_type, value="generic").pack(anchor="w")

        # Start button
        ttk.Button(frame, text="Start Therapy", command=self._start_therapy).pack(fill="x", pady=10, ipady=10)

    def _upload_pdf(self):
        file_path = filedialog.askopenfilename(filetypes=[("PDF Files", "*.pdf")])
        if file_path:
            self.history_path = file_path
            self._preview_pdf(file_path)

    def _preview_pdf(self, path):
        self.history_label.config(text=f"History: {Path(path).name}")
        doc = fitz.open(path)
        page = doc.load_page(0)
        pix = page.get_pixmap()
        mode = "RGBA" if pix.alpha else "RGB"
        img = Image.frombytes(mode, (pix.width, pix.height), pix.samples)
        img.thumbnail((500, 400))
        photo = ImageTk.PhotoImage(img)
        self.preview_label.configure(image=photo, text="")
        self.preview_label.image = photo

    def _start_therapy(self):
        # Placeholder callback for starting therapy session
        print("Starting therapy")
        print("Image path:", self.image_path)
        print("Image type:", self.image_type.get())
        print("PDF path:", self.history_path)
        messagebox.showinfo("Therapy", "Therapy started. See console output for details.")


if __name__ == "__main__":
    patient = {
        "name": "Alex Smith",
        "age": "73"
    }

    root = tk.Tk()
    app = ReminiscenceTherapyGUI(root, patient)
    root.mainloop()
