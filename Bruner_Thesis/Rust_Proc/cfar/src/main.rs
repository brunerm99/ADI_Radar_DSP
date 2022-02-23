fn calculate_threshold(data : Vec<f32>, num_guard_cells : u32, num_ref_cells : u32, bias : f32) {
    let size = data.len();

    let threshold : Vec<f32> = data.iter()
        .map(|&x| x + 1.0)
        .collect::<Vec<f32>>();

    for value in threshold {
        println!("Value: {}", value);
    }

}

fn main() {
    let num_guard_cells : u32 = 10;
    let num_ref_cells : u32 = 30;
    let bias : f32 = 2.0;
    let method = "greatest";

    let data : Vec<f32> = vec![4.1, 4.3, 3.5, 3.6, 4.6, 8.2, 9.9, 3.2, 2.3, 3.3];
    calculate_threshold(data, num_guard_cells, num_ref_cells, bias);
}
